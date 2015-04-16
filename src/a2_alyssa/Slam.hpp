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

#include "math/point.hpp"
#include "MagicNumbers.hpp"

class Slam
{
    private:
        Particles particles_;
        OccupancyGridMapper *grid_mapper_;

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

        void addPose(int left_ticks, int right_ticks, int64_t utime);
    public:
        Slam(OccupancyGridMapper *gm);
        ~Slam();
        void setLCM(lcm::LCM *lcm_t);

        void addMotorFeedback(maebot_motor_feedback_t input_feedback);
        void updateParticles();
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
