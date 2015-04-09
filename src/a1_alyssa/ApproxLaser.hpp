#ifndef APPROX_LASER_HPP
#define APPROX_LASER_HPP

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

/* class to figure out approximately where laser scan originated from */
class ApproxLaser
{
    private:
        /* keep track of past five poses */
        std::deque<maebot_pose_t> poses_;

    public:
        /*ApproxLaser(MovingLaser *laser_t){
            moving_laser = laser_t;
        }*/

        ~ApproxLaser(){}

        /* find the two points the scan originated between */
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
