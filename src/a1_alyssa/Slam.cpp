#include "Slam.hpp"
#include <cassert>

Slam::Slam(OccupancyGridMapper *gm, lcm::LCM *lcm_t, location *loc_t) :
    grid_mapper_(gm),
    scan_received_(false),
    lcm(lcm_t),
    loc_(loc_t),
    prev_time(0)
{
    pthread_mutex_init(&slam_mutex_, NULL);
    pthread_mutex_init(&poses_mutex_, NULL);
    pthread_mutex_init(&scans_mutex_, NULL);
    pthread_mutex_init(&path_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);

    left_prev_ticks = -1;
    right_prev_ticks = -1;
    prev_odometry_pose.x = prev_pose.x = 0;
    prev_odometry_pose.y = prev_pose.y = 0;
    prev_odometry_pose.theta = prev_pose.theta = 0;
    prev_odometry_pose.utime = 0; 
    poses_.push_back(prev_odometry_pose);
}

Slam::~Slam()
{
}

bool Slam::posesEmpty()
{
    return poses_.empty();
}

void Slam::addMotorFeedback(maebot_motor_feedback_t input_feedback)
{
    if(left_prev_ticks == -1 || right_prev_ticks == -1)
    {
        left_prev_ticks = input_feedback.encoder_left_ticks;
        right_prev_ticks = input_feedback.encoder_right_ticks;
        assert(!poses_.empty());
        return;
    }
    addPose(input_feedback.encoder_left_ticks, input_feedback.encoder_right_ticks, input_feedback.utime);
}

void Slam::addScan(maebot_laser_scan_t laser_scan)
{
    pthread_mutex_lock(&scans_mutex_);
    //std::cout << "add scan mutex lock" << std::endl;
    scans_.push(laser_scan);
    pthread_mutex_unlock(&scans_mutex_);
    //std::cout << "add scan mutex unlock" << std::endl;
    prev_time = laser_scan.utime;
    scan_received_ = true;
    
}

/* absolute pose, not delta pose */
void Slam::addPose(int left_ticks, int right_ticks, int64_t utime)
{
    if(left_ticks == left_prev_ticks && right_ticks == right_prev_ticks)
    {
        maebot_pose_t newPose = prev_pose;
        newPose.utime = utime;
        pthread_mutex_lock(&poses_mutex_);
        //std::cout << "addPose mutex lock" << std::endl;
        poses_.push_back(newPose);
        pthread_mutex_unlock(&poses_mutex_);
        //std::cout << "addPose mutex unlock" << std::endl;
        return;
    }

    double dtheta, s, alpha;
    double x_prev = prev_odometry_pose.x;
    double y_prev = prev_odometry_pose.y;
    double theta_prev = prev_odometry_pose.theta;

    int sL_i = left_ticks - left_prev_ticks;
    int sR_i = right_ticks - right_prev_ticks;

    // increment left and right starting tick
    left_prev_ticks = left_ticks;
    right_prev_ticks = right_ticks;

    //convert ticks to meters
    double sL_f = sL_i/4800.0;
    double sR_f = sR_i/4800.0;

    s = (sR_f + sL_f)/2;
    dtheta = (sR_f - sL_f)/0.08;
    alpha = dtheta/2;
    maebot_pose_t nextPose;
    nextPose.x = s*(cos(theta_prev + alpha)) + x_prev;
    nextPose.y = s*(sin(theta_prev + alpha)) + y_prev;
    nextPose.theta = dtheta + theta_prev;
    nextPose.utime = utime;
    prev_odometry_pose = nextPose;

    pthread_mutex_lock(&poses_mutex_);
    //std::cout << "addPose2 mutex lock" << std::endl;
    poses_.push_back(nextPose);
    pthread_mutex_unlock(&poses_mutex_);
    //std::cout << "addPose2 mutex unlock" << std::endl;
}

void Slam::pushFirstScan()
{
    pthread_mutex_lock(&scans_mutex_);
    //std::cout << "push First Scan mutex lock" << std::endl;
    maebot_laser_scan_t first = scans_.front();
    pthread_mutex_unlock(&scans_mutex_);
    //std::cout << "push First scan mutex unlock" << std::endl;
    grid_mapper_->addLaserScan(first);

    maebot_pose_t mostProbable = particles_.mostProbable();
    grid_mapper_->addPose(mostProbable);
}

maebot_laser_scan_t Slam::updateParticles()
{
    assert(scan_received_);
    pthread_mutex_lock(&scans_mutex_);
    //std::cout << "lock scans mutex update particles" << std::endl;
    maebot_laser_scan_t nextScan = scans_.front();
    scans_.pop();
    if(scans_.empty())
    {
        scan_received_ = false;
    }
    pthread_mutex_unlock(&scans_mutex_);
    //std::cout << "unlock scans mutex update particles" << std::endl;

    // if only 1 pose, can't call findLeftRightPoses
    pthread_mutex_lock(&poses_mutex_);
    //std::cout << "lock poses mutex update particles" << std::endl;
    if(poses_.size() < 2)
    {
        pthread_mutex_unlock(&poses_mutex_);
        //std::cout << "unlock poses mutex update particles" << std::endl;
        return nextScan;
    }
    assert(prev_pose.utime <= nextScan.utime);
    LaserScanRange lsr = particles_.getLaserScan(prev_pose, nextScan, poses_, mB);
    pthread_mutex_unlock(&poses_mutex_);
    //std::cout << "unlock poses mutex update particles" << std::endl;
    if(!lsr.valid)
    {
        return nextScan;
    }

    // if the maebot hasn't moved, update the particle times and publish, but don't actually move particles
    if(lsr.end_pose.x == lsr.start_pose.x && lsr.end_pose.y == lsr.end_pose.y)
    {
        particles_.updateTimes(lsr.end_pose.utime);
        prev_pose.utime = lsr.end_pose.utime;
        publish();
        return nextScan;
    }

    //std::cout << "update Particles\n";
    //assert not nan
    assert(prev_pose.x == prev_pose.x);
    assert(prev_pose.y == prev_pose.y);
    assert(lsr.end_pose.x == lsr.end_pose.x);
    assert(lsr.end_pose.y == lsr.end_pose.y);

    double delta_x = lsr.end_pose.x - prev_pose.x;
    double delta_y = lsr.end_pose.y - prev_pose.y;
    double delta_theta = eecs467::angle_diff(lsr.end_pose.theta, prev_pose.theta);
    assert(delta_x == delta_x);
    assert(delta_y == delta_y);

    particles_.updateParticles(delta_x, delta_y, delta_theta, prev_pose.theta, &grid_mapper_->getOccupancyGrid(), lsr);
    prev_pose = lsr.end_pose;
    publish();
    particles_.resample();

    return nextScan;
}

void Slam::publish()
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        maebot_pose_t pose = particles_.toPose(i);
        lcm->publish("MAEBOT_PARTICLE_GUI", &pose);
    }

    for(int i = 0; i < bfs_result.size(); i++)
    {
        maebot_pose_t pose;
        eecs467::Point<int> p(grid_mapper_->toX(bfs_result[i]), grid_mapper_->toY(bfs_result[i]));
        auto pp = eecs467::grid_position_to_global_position(p, grid_mapper_->bfs_);
        pose.x = pp.x;
        pose.y = pp.y;
        lcm->publish("MAEBOT_PATH_GUI", &pose);
    }
    //lcm->publish("MAEBOT_PARTICLE_GUI", &poses_.back());
    
    // publish particle to gui
    maebot_pose_t mostProbable = particles_.mostProbable();
    //lcm->publish("MAEBOT_POSE_GUI", &mostProbable);

    grid_mapper_->lockMapperMutex();
    grid_mapper_->addPose(mostProbable);
   
    pthread_mutex_lock(&loc_->move_mutex);
    loc_->x = mostProbable.x;
    loc_->y = mostProbable.y;
    loc_->theta = mostProbable.theta;
    pthread_mutex_unlock(&loc_->move_mutex);

    if(!grid_mapper_->laserScansEmpty())
    {
        grid_mapper_->signal();
    }
    grid_mapper_->unlockMapperMutex();
    //lcm->publish("MAEBOT_POSE_BEST", &mostProbable);
    //std::cout << "sent particles\n";
}

maebot_pose_t Slam::mostProbableParticle()
{
    return particles_.mostProbable();
}

bool Slam::scanReceived()
{
    return scan_received_;
}

void Slam::lockSlamMutex()
{
    pthread_mutex_lock(&slam_mutex_);
    //std::cout << "slam mutex locked" << std::endl;
}

void Slam::unlockSlamMutex()
{
    pthread_mutex_unlock(&slam_mutex_);
}

void Slam::signal()
{
    pthread_cond_signal(&cv_);
}

void Slam::wait()
{
    pthread_cond_wait(&cv_, &slam_mutex_);
}
