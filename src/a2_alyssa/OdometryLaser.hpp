#ifndef ODOMETRY_LASER_HPP
#define ODOMETRY_LASER_HPP

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>
#include <deque>
#include <queue>
#include <iostream>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "MovingLaser.hpp"

struct OdometryLaserRange
{
    bool valid;
    maebot_pose_t start_pose;
    maebot_pose_t end_pose;
    maebot_laser_scan_t scan;
};

class OdometryLaser
{
    private:
        /* keep track of past five odometry data*/
        std::deque<maebot_pose_t> odometries_;

    public:
        OdometryLaser(){}

        ~OdometryLaser(){}

        LaserScanRange findPts(const maebot_laser_scan_t *scan);

        /* add pose to deque (and delete oldest pose */
        void addPose(const maebot_pose_t *newPose);

        bool containsPoses()
        {
            return poses_.size() > 0;
        }

        int posesSize()
        {
            return poses_.size();
        }
};

#endif
