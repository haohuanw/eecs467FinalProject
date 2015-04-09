#ifndef SLAM_HPP
#define SLAM_HPP

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>
#include <queue>
#include <iostream>

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
#include "OccupancyGridMapper.hpp"
#include "Particles.hpp"
//#include "Path.h"

#include "math/point.hpp"
#include "MagicNumbers.hpp"

struct location {
    double theta;
    double x;
    double y;
    pthread_mutex_t move_mutex;
};

class Slam
{
    private:
        Particles particles_;
        OccupancyGridMapper *grid_mapper_;
        location *loc_;
        //Path path;

        std::vector<maebot_pose_t> poses_;
        pthread_mutex_t poses_mutex_;

        std::queue<maebot_laser_scan_t> scans_;
        pthread_mutex_t scans_mutex_;

        pthread_mutex_t slam_mutex_;
        pthread_cond_t cv_;
        bool scan_received_;

        lcm::LCM *lcm;

        int left_prev_ticks;
        int right_prev_ticks;
        int64_t prev_time;
        maebot_pose_t prev_pose;
        maebot_pose_t prev_odometry_pose;
        maebot_pose_t mB[2];

        void addPose(int left_ticks, int right_ticks, int64_t utime);

    public:
        Slam(OccupancyGridMapper *gm, lcm::LCM *lcm_t, location *loc_t);
        ~Slam();
        
        pthread_mutex_t path_mutex_;
        std::vector<int> bfs_result;
        maebot_pose_t mostProbableParticle();

        void addMotorFeedback(maebot_motor_feedback_t input_feedback);
        maebot_laser_scan_t updateParticles();
        void pushFirstScan();
        void addScan(maebot_laser_scan_t scan);
        void publish();
        
        bool posesEmpty();
        bool scanReceived();
        
        void lockSlamMutex();
        void unlockSlamMutex();
        void signal();
        void wait();
};

#endif
