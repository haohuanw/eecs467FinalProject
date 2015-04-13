#ifndef MAEBOT_HANDLERS_HPP
#define MAEBOT_HANDLERS_HPP

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <iostream>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "MagicNumbers.hpp" 
#include "ApproxLaser.hpp"
#include "OccupancyGridMapper.hpp"
#include "Slam.hpp"

class MaebotLCMHandler
{
    private:
        OccupancyGridMapper *grid_mapper_;
        Slam *slam_;

    public:
        MaebotLCMHandler(OccupancyGridMapper *grid_mapper_t, Slam *slam_t) :
            grid_mapper_(grid_mapper_t),
            slam_(slam_t) { }
        
        ~MaebotLCMHandler(){}

        void handlePose(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_pose_t *msg)
        {
            grid_mapper_->lockPosesMutex();
            grid_mapper_->lockMapperMutex();
            grid_mapper_->addPose(*msg);
            if(!grid_mapper_->laserScansEmpty())
            {
                grid_mapper_->signal();
            }
            grid_mapper_->unlockPosesMutex();
            grid_mapper_->unlockMapperMutex();
        }

        void handleLaserScan(const lcm::ReceiveBuffer *rbuf,
                             const std::string& channel,
                             const maebot_laser_scan_t *msg)
        {
            if(task1)
            {
                grid_mapper_->lockLaserScansMutex();
                grid_mapper_->lockMapperMutex();

                grid_mapper_->addLaserScan(*msg);
                if(!grid_mapper_->posesEmpty())
                    grid_mapper_->signal();

                grid_mapper_->unlockLaserScansMutex();
                grid_mapper_->unlockMapperMutex();

            }
            else if(task2)
            {
                slam_->addScan(*msg);
                std::cout << "scan received\n";
            }
        }

        void handleMotorFeedback(const lcm::ReceiveBuffer *rbuf,
                                 const std::string& channel,
                                 const maebot_motor_feedback_t *msg)
        {
            slam_->lockSlamMutex();
            slam_->addMotorFeedback(*msg);
            if(slam_->scanReceived())
            {
                slam_->signal();
            }
            slam_->unlockSlamMutex();
        }
};

#endif
