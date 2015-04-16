#include "Slam.hpp"
#include <cassert>

Slam::Slam(OccupancyGridMapper *gm) :
    grid_mapper_(gm),
    scan_received_(false),
    prev_time(0)
{
    pthread_mutex_init(&poses_mutex_, NULL);
    pthread_mutex_init(&slam_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);
    pthread_mutex_init(&scans_mutex_, NULL);

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

void Slam::setLCM(lcm::LCM *lcm_t)
{
    lcm = lcm_t;
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
        //poses_[0].utime = input_feedback.utime;
        return;
    }
    pthread_mutex_lock(&poses_mutex_);
    addPose(input_feedback.encoder_left_ticks, input_feedback.encoder_right_ticks, input_feedback.utime);
    pthread_mutex_unlock(&poses_mutex_);
}

void Slam::addScan(maebot_laser_scan_t laser_scan)
{
    pthread_mutex_lock(&scans_mutex_);
    //double delta_t = laser_scan.utime - prev_time;
    //std::cout << "scans sanity check: " << delta_t << std::endl;
    scans_.push(laser_scan);
    prev_time = laser_scan.utime;
    scan_received_ = true;
    pthread_mutex_unlock(&scans_mutex_);
}

/* absolute pose, not delta pose */
void Slam::addPose(int left_ticks, int right_ticks, int64_t utime)
{
    if(left_ticks == left_prev_ticks && right_ticks == right_prev_ticks)
    {
        maebot_pose_t newPose = prev_pose;
        newPose.utime = utime;
        poses_.push_back(newPose);
        //std::cout << "left_ticks:  " << left_ticks << " left_ticks_prev:  " << left_prev_ticks << std::endl;
        //std::cout << "right_ticks: " << right_ticks << " right_ticks_prev: " << right_prev_ticks << std::endl;
        return;
    }
    //std::cout << "left ticks:  " << left_ticks << std::endl;
    //std::cout << "right ticks: " << right_ticks << std::endl;
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
    //std::cout << "nextPose: (" << nextPose.x << ", " << nextPose.y << ", " << nextPose.theta << ", " << nextPose.utime << ")\n";

    poses_.push_back(nextPose);
}

void Slam::updateParticles()
{
    pthread_mutex_lock(&scans_mutex_);
    //std::cout << "lock1" << std::endl;
    assert(scan_received_);
    maebot_laser_scan_t nextScan = scans_.front();
    scans_.pop();
    if(scans_.empty()) { scan_received_ = false; }
    pthread_mutex_unlock(&scans_mutex_);

    // if only 1 pose, can't call findLeftRightPoses
    if(poses_.size() < 2)
    {
        return;
    }
    pthread_mutex_lock(&poses_mutex_);
    //std::cout << "lock2" << std::endl;
    //std::cout << "prevPose: (" << prev_pose.x << ", " << prev_pose.y << ", " << prev_pose.theta << ")\n";
    assert(prev_pose.utime <= nextScan.utime);
    LaserScanRange lsr = particles_.getLaserScan(prev_pose, nextScan, poses_);
    //std::cout << "here" << std::endl;
    //std::cout << "deltas!!!: (" << lsr.end_pose.x << ", " << lsr.end_pose.y << ", " << lsr.end_pose.theta << ")\n";
    pthread_mutex_unlock(&poses_mutex_);
    if(!lsr.valid)
    {
        return;
    }

    // if the maebot hasn't moved, update the particle times and publish, but don't actually move particles
    if(lsr.end_pose.x == lsr.start_pose.x && lsr.end_pose.y == lsr.end_pose.y)
    {
        particles_.updateTimes(lsr.end_pose.utime);
        prev_pose.utime = lsr.end_pose.utime;
        publish();
        return;
    }

    std::cout << "update Particles\n";
    //assert not nan
    assert(prev_pose.x == prev_pose.x);
    assert(prev_pose.y == prev_pose.y);
    assert(lsr.end_pose.x == lsr.end_pose.x);
    assert(lsr.end_pose.y == lsr.end_pose.y);

    double delta_x = lsr.end_pose.x - prev_pose.x;
    double delta_y = lsr.end_pose.y - prev_pose.y;
    double delta_theta = eecs467::angle_diff(lsr.end_pose.theta, prev_pose.theta);
    //std::cout << "delta t: " << lsr.end_pose.utime - prev_pose.utime << std::endl;
    //std::cout << "here2" << std::endl;
    assert(delta_x == delta_x);
    assert(delta_y == delta_y);

    particles_.updateParticles(delta_x, delta_y, delta_theta, prev_pose.theta, &grid_mapper_->getOccupancyGrid(), lsr);
    prev_pose = lsr.end_pose;
    publish();
    particles_.resample();
    //std::cout << "here3" << std::endl;
}

void Slam::publish()
{
    //particles_.print();
    //std::cout << "in publish" << std::endl;
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        maebot_pose_t pose = particles_.toPose(i);
        //std::cout << pose.x << ", " << pose.y << std::endl;
        lcm->publish("MAEBOT_PARTICLE_GUI", &pose);
    }
    //maebot_pose_t particle = particles_.toPose(0);
    //std::cout << "first particle: (" << particle.x << ", " << particle.y << ", " << particle.theta << ")\n";
    maebot_pose_t mostProbable = particles_.mostProbable();
    lcm->publish("MAEBOT_POSE_GUI", &mostProbable);
    lcm->publish("MAEBOT_POSE_BEST_GUI", &mostProbable);
    //std::cout << "sent particles\n";
}

bool Slam::scanReceived()
{
    return scan_received_;
}

void Slam::lockSlamMutex()
{
    pthread_mutex_lock(&slam_mutex_);
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
