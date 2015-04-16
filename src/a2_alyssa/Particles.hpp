#ifndef PARTICLES_HPP
#define PARTICLES_HPP

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <deque>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "lcmtypes/maebot_occupancy_grid_t.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "ApproxLaser.hpp"
#include "MagicNumbers.hpp"
#include "MovingLaser.hpp"

#include "math/point.hpp"
#include "math/gsl_util_rand.h"

struct Particle
{
    double x;
    double y;
    double theta;
    int64_t utime;
    double probability;
};

struct comp
{
    bool operator()(Particle a, Particle b) { return a.probability < b.probability; }
};

class Particles
{
    private:
        std::vector<Particle> particles_;
        Particle most_likely_;
        gsl_rng *r;
    public:
        Particles();
        ~Particles();
        maebot_pose_t toPose(int index);
        maebot_pose_t mostProbable();
        
        void print();
        void printProbabilities();
        void updateTimes(int64_t utime);
        void updateParticles(double delta_x, double delta_y, double delta_theta, double prev_theta, eecs467::OccupancyGrid *grid, LaserScanRange scan);

        void moveRandom(eecs467::OccupancyGrid *grid, LaserScanRange lsr, double mean_x, double mean_y, double mean_theta, double prev_theta);
        void moveRandomSingle(eecs467::OccupancyGrid *grid, LaserScanRange laser_scan_range, double delta_s, double alpha, double theta_alpha, int index);

        //void calculateProbability(occupancy_grid_t *grid, maebot_laser_scan_t *scan);
        void calculateProbabilitySingle(eecs467::OccupancyGrid *grid, LaserScanRange scan, int index);
        void rotateParticle(double theta, int index);
        void moveParticle(double s, int index);
        int countBlacks(eecs467::OccupancyGrid *grid, double s_x, double s_y, double e_x, double e_y, double inc);

        void normalizeProbabilities();
        void exponentiate();
        void subtractProbabilities(double max);
        int findLargestProbability();
        double sumProbabilities();
        void divideProbabilities(double sum);
        
        void resample();
        
        LaserScanRange getLaserScan(maebot_pose_t poseA, maebot_laser_scan_t scanB, std::vector<maebot_pose_t>& poses);
        std::vector<maebot_pose_t> findLeftRightPoses(int64_t time, std::vector<maebot_pose_t>& poses);
};

#endif
