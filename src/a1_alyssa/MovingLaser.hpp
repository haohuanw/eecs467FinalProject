#ifndef MOVING_LASER_HPP
#define MOVING_LASER_HPP

#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "math/angle_functions.hpp"

/* store lidar scans and associated poses where they originated */
struct LaserScan
{
    bool valid;
    std::vector<maebot_pose_t> origins;
    maebot_laser_scan_t scan;
    maebot_pose_t end_pose;
};

/* store the lidar scan and the two poses it originated between */
struct LaserScanRange
{
    bool valid;
    maebot_pose_t start_pose;
    maebot_pose_t end_pose;
    maebot_laser_scan_t scan;
    LaserScanRange(bool valid_, maebot_pose_t start, maebot_pose_t end, maebot_laser_scan_t scan_t)
        : valid(valid_), start_pose(start), end_pose(end), scan(scan_t) {}
};

/* functor that figure out where each individual laser originated from */
class MovingLaser
{
    public:
        LaserScan findOrigin(LaserScanRange approx_scan);
        maebot_pose_t findOriginSingle(int64_t, maebot_pose_t, maebot_pose_t);
};

#endif
