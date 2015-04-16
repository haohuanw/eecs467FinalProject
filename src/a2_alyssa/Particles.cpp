#include "Particles.hpp"

#include <cassert>

Particles::Particles()
{
    // construct random environment
    r = gslu_rand_rng_alloc();

    // construct individual particles
    double increment = 1.0/NUM_PARTICLES;
    Particle particle = {0, 0, 0, 0, increment};
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_.push_back(particle);
    }
    most_likely_ = particles_[0];
}

Particles::~Particles()
{
}

maebot_pose_t Particles::toPose(int index)
{
    maebot_pose_t retval;
    retval.x = particles_[index].x;
    //std::cout << "particles x: " << particles_[index].x << std::endl;
    retval.y = particles_[index].y;
    retval.theta = particles_[index].theta;
    retval.utime = particles_[index].utime;

    assert(retval.x == retval.x);

    return retval;
}

void Particles::updateTimes(int64_t utime)
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_[i].utime = utime;
    }
    most_likely_ = particles_[0];
}

maebot_pose_t Particles::mostProbable()
{
    maebot_pose_t retval;
    retval.x = most_likely_.x;
    retval.y = most_likely_.y;
    retval.theta = most_likely_.theta;
    retval.utime = most_likely_.utime;
    return retval;
}

void Particles::updateParticles(double delta_x, double delta_y, double delta_theta, double prev_theta, eecs467::OccupancyGrid *grid, LaserScanRange scan)
{
    //std::cout << "deltas: (" << delta_x << ", " << delta_y << ", " << delta_theta << ")\n";
    moveRandom(grid, scan, delta_x, delta_y, delta_theta, prev_theta);
    most_likely_ = particles_[findLargestProbability()];
    //std::cout << "here 4" << std::endl;
    normalizeProbabilities();
    //std::cout << "here 5" << std::endl;
    //resample();
    //std::cout << "here 6" << std::endl;
}

void Particles::print()
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        std::cout << particles_[i].x << ", " << particles_[i].y << ", " << particles_[i].theta << std::endl;
    }
}

void Particles::moveRandom(eecs467::OccupancyGrid *grid, LaserScanRange lsr, double delta_x, double delta_y, double delta_theta, double prev_theta)
{
    assert(delta_x == delta_x);
    assert(delta_y == delta_y);

    double delta_s = sqrt((delta_x * delta_x) + (delta_y * delta_y));
    double alpha = eecs467::angle_diff(atan2(delta_y, delta_x), prev_theta);
    double theta_alpha = eecs467::angle_diff(delta_theta, alpha);

    //std::cout << "deltas after: (" << alpha << ", " << delta_s << ", " << theta_alpha << ")\n";

    //std::vector<double> random_nums;
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        //random_nums.push_back(gslu_rand_gaussian(r, delta_s, 0.9*delta_s));
        moveRandomSingle(grid, lsr, delta_s, alpha, theta_alpha, i);
        //std::cout << particles_[i].x << " " <<  particles_[i].y << " " <<  particles_[i].theta << std::endl;
    }

    /*int count = 0;
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        for(int j = 0; j < NUM_PARTICLES; j++)
        {
            if(i==j) {continue;}
            //assert(random_nums[i] != random_nums[j]);
            assert(particles_[i].x != particles_[j].x &&
                   particles_[i].y != particles_[j].y &&
                   particles_[i].theta != particles_[j].theta);
            if(particles_[i].x == particles_[j].x ||
               particles_[i].y == particles_[j].y ||
               particles_[i].theta == particles_[j].theta)
            {
                count++;
            }
        }
    }
    std::cout << "count: " << count << std::endl;*/
}

void Particles::moveRandomSingle(eecs467::OccupancyGrid *grid, LaserScanRange laser_scan_range, double delta_s, double alpha, double theta_alpha, int index)
{
    //double new_delta_s = gslu_rand_gaussian(r, delta_s, 0.9*delta_s);
    //double new_delta_alpha = gslu_rand_gaussian(r, alpha, 3.14);
    //double new_delta_theta_alpha = gslu_rand_gaussian(r, theta_alpha, 3.14);
    
    //assert(new_delta_s == new_delta_s);
    //assert(delta_s == delta_s);

    laser_scan_range.start_pose = toPose(index);

    rotateParticle(eecs467::wrap_to_pi(gslu_rand_gaussian(r, alpha, (3.141592/60 + alpha/2))), index);
    moveParticle(gslu_rand_gaussian(r, delta_s, sqrt(0.005*delta_s)), index);
    //moveParticle(gslu_rand_gaussian(r, 0, delta_s), index);
    //moveParticle(gslu_rand_gaussian(r, delta_s, 5), index);
    rotateParticle(eecs467::wrap_to_pi(gslu_rand_gaussian(r, theta_alpha, (3.141592/60 + alpha/2))), index);
    
    //rotateParticle(new_delta_alpha, index);
    //moveParticle(new_delta_s, index);
    //rotateParticle(new_delta_theta_alpha, index);
    particles_[index].utime = laser_scan_range.end_pose.utime;

    laser_scan_range.end_pose = toPose(index);

    // calculate probability of new particle
    calculateProbabilitySingle(grid, laser_scan_range, index);
}

void Particles::rotateParticle(double theta, int index)
{
    particles_[index].theta = eecs467::wrap_to_2pi(particles_[index].theta + theta);
}

void Particles::moveParticle(double s, int index)
{
    particles_[index].x += s * cos(particles_[index].theta) + gslu_rand_gaussian(r, 0, 0.1);
    particles_[index].y += s * sin(particles_[index].theta) + gslu_rand_gaussian(r, 0, 0.1);

    assert(s == s);
    assert(particles_[index].x == particles_[index].x);
    assert(particles_[index].y == particles_[index].y);
}

int Particles::countBlacks(eecs467::OccupancyGrid *grid, double s_x, double s_y, double e_x, double e_y, double inc)
{
    double a = atan2(e_y - s_y, e_x - s_x);
    double dist = sqrt((e_x - s_x)*(e_x - s_x)+(e_y - s_y)*(e_y - s_y));
    if(dist == 0)
    {
        return 0;
    }
    int n = ceil(dist / inc);
    inc = dist/n;
    double i_x = cos(a) * inc,
           i_y = sin(a) * inc,
           p_x = s_x,
           p_y = s_y;
    int count = 0;
           
    for(int i = 0; i <= n; i++)
    {
        int v;
        eecs467::Point<double> p(p_x, p_y);
        auto p_origin = eecs467::global_position_to_grid_cell(p, *grid);
        v = (int)grid->logOdds(p_origin.x, p_origin.y);
        if(v > 0) { count++; }
            
        p_x += i_x;
        p_y += i_y;
    }
    return count;
}
void Particles::calculateProbabilitySingle(eecs467::OccupancyGrid *grid, LaserScanRange scan, int index)
{
    MovingLaser ml;
    LaserScan ls = ml.findOrigin(scan);
    if(ls.origins.size() != ls.scan.ranges.size()) { std::cout << "uh oh\n"; exit(1); }
    particles_[index].probability = 0;
    for(uint i = 0; i < ls.origins.size(); i++)
    {
        auto a = -ls.scan.thetas[i] + ls.origins[i].theta;
        auto x = ls.origins[i].x + ls.scan.ranges[i] * cos(a);
        auto y = ls.origins[i].y + ls.scan.ranges[i] * sin(a);
        int numBlacks = countBlacks(grid, ls.origins[i].x, ls.origins[i].y, x, y, 0.05);
        
        particles_[index].probability -= numBlacks*6;
        //eecs467::Point<double> point(x, y);
        //eecs467::Point<int> cellPoint = global_position_to_grid_cell(point, *grid);
        //auto v = grid->logOdds(cellPoint.x, cellPoint.y);
        //particles_[index].probability += v*0.1;
        eecs467::Point<double> point(x, y);
        eecs467::Point<int> cellPoint = global_position_to_grid_cell(point, *grid);
        auto v = grid->logOdds(cellPoint.x, cellPoint.y);
        if(v < 0)
        {
            particles_[index].probability -= 8;
        }
        else if(v > 0)
        {
            particles_[index].probability -= 2;
        }
        else
        {
            particles_[index].probability -= 10;
        }
    }
}

void Particles::normalizeProbabilities()
{
    int index_max = findLargestProbability();
    //std::cout << "normalize\n";
    //printProbabilities();
    subtractProbabilities(particles_[index_max].probability);
    exponentiate();
    double sum = sumProbabilities();
    if(sum == 0) { std::cout << "WHAT THE FUCK IS THIS SHIT" << std::endl; exit(1); }
    divideProbabilities(sum);
}

void Particles::exponentiate()
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_[i].probability = exp(particles_[i].probability);
    }
}

void Particles::subtractProbabilities(double max)
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_[i].probability -= max;
    }

    return;
}

int Particles::findLargestProbability()
{
    int index = 0;
    for(int i = 1; i < NUM_PARTICLES; i++)
    {
        if(particles_[i].probability > particles_[index].probability)
            index = i;
    }
    return index;
}

double Particles::sumProbabilities()
{
    double sum = 0;
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        sum += particles_[i].probability;
    }
    return sum;
}

void Particles::divideProbabilities(double sum)
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        particles_[i].probability /= sum;
    }
}

void Particles::printProbabilities()
{
    for(int i = 0; i < NUM_PARTICLES; i++)
    {
        std::cout << particles_[i].probability << std::endl;
    }
}

void Particles::resample()
{
    // write own functor for comparing probs
    comp comparator;
    std::sort(particles_.begin(), particles_.end(), comparator);
    //printProbabilities();
    //double increment = 1.0/NUM_PARTICLES;
    std::vector<Particle> resampled;
    resampled.reserve(NUM_PARTICLES);
    /*for(double i = 0, index = 0, newvecindex = 0; i < 1; i+= increment, newvecindex++)
    {
        bool found = false;
        for(int j = index; j < NUM_PARTICLES; j++)
        {
            if(particles_[j].probability > i)
            {
                index = (j == 0 ? 0:j-1);
                found = true;
                break;
            }
        }
        if(!found) { index = NUM_PARTICLES - 1; }
        resampled[newvecindex] = particles_[index];
        resampled[newvecindex].probability = increment;
    }*/
    double c = particles_[0].probability;
    int i = 0;
    double U = 0;
    for(int m = 0; m < NUM_PARTICLES; m++, U += (1.0/NUM_PARTICLES))
    {
        while(U > c)
        {
            i++;
            c = c + particles_[i].probability;
        }
        resampled.push_back(particles_[i]);
    }
    particles_ = resampled;
    //print();
    //std::cout << "------------------------------------------------------------------------------------" << std::endl;
}

/* find position of laser scan and positions of individual laser beams within scan */

LaserScanRange Particles::getLaserScan(maebot_pose_t poseA, maebot_laser_scan_t scanB, std::vector<maebot_pose_t>& poses)
{
    std::vector<maebot_pose_t> mB = findLeftRightPoses(scanB.utime, poses);
    if(mB.size() == 1)
    {
        LaserScanRange lsr(true, poseA, poseA, scanB);
        return lsr;
    }
    MovingLaser moving_laser;
    
    // find the pose of the last laser scan
    assert(mB[0].utime <= mB[1].utime);
    maebot_pose_t poseB = moving_laser.findOriginSingle(scanB.utime, mB[0], mB[1]);
    LaserScanRange lsr(true, poseA, poseB, scanB);
    assert(poseA.utime <= poseB.utime);
    assert(lsr.end_pose.x == lsr.end_pose.x);
    assert(lsr.end_pose.y == lsr.end_pose.y);
    return lsr;
}

std::vector<maebot_pose_t> Particles::findLeftRightPoses(int64_t time, std::vector<maebot_pose_t>& poses)
{
    //std::cout << "poses size: " << poses.size() << std::endl;
    //std::cout << "poses[0]: " << (poses.end()-1)->utime << ", " << (poses.end()-1)->x << ", " << (poses.end()-1)->y << ", " << (poses.end()-1)->theta << ")" << std::endl;
    //std::cout << "poses[1]: " << (poses.end()-2)->utime << std::endl;
    //std::cout << "time:     " << time << std::endl;
    std::vector<maebot_pose_t> m;
    assert(!poses.empty());
    if(poses.size() == 1)
    {
        m.push_back(poses[0]);
        return m;
    }
    //std::cout << "scan time: " << time << std::endl;
    for(int i = (int)(poses.size())-1; i >= 0; --i)
    {
        //std::cout << "pose time[" << i << "]: " << poses[i].utime << std::endl;
 //       assert(it->utime);
        assert(time);
        if(poses[i].utime < time)
        {
            m.push_back(poses[i]);
            if(i+1 < (int)poses.size())
                m.push_back(poses[i+1]);
            return m;
        }
    }
    assert(false);
    return m;
}
