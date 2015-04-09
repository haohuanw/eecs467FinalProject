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
//#include <time.h>

#include "common/timestamp.h"

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
#include "state.hpp"


bool reachGoal(state_t *state) {
    return (abs(state->loc.x - state->next_x) < SAME_REGION_R && 
            abs(state->loc.y - state->next_y) < SAME_REGION_R);
}

bool rightAngle(state_t *state, double &angle) {
    if (state->next_x != state->loc.x) 
        angle = atan2((state->next_y - state->loc.y),(state->next_x - state->loc.x));
    else if (state->next_y > state->loc.y)
        angle = 3.14/2.0;
    else 
        angle = -3.14/2.0;
    
    std::cout << "angles: now " << state->loc.theta << " to " << angle << std::endl;
    return (abs(eecs467::angle_diff(eecs467::wrap_to_2pi(state->loc.theta), 
                    eecs467::wrap_to_2pi(angle))) < SAME_ANGLE_D);
}

class MaebotLCMHandler
{
    private:
        OccupancyGridMapper *grid_mapper_;
        Slam *slam_;
        state_t *state_;

    public:
        MaebotLCMHandler(OccupancyGridMapper *grid_mapper_t, Slam *slam_t, 
                         state_t *state_t) :
            grid_mapper_(grid_mapper_t),
            slam_(slam_t),
            state_(state_t) { }
        
        ~MaebotLCMHandler(){}

        /*void handlePose(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_pose_t *msg)
        {
            assert(channel == "MAEBOT_POSE_BEST");
            grid_mapper_->lockPosesMutex();
            grid_mapper_->lockMapperMutex();
            grid_mapper_->addPose(*msg);
            if(!grid_mapper_->laserScansEmpty())
            {
                grid_mapper_->signal();
            }
            grid_mapper_->unlockPosesMutex();
            grid_mapper_->unlockMapperMutex();
        }*/

        void handleLaserScan(const lcm::ReceiveBuffer *rbuf,
                             const std::string& channel,
                             const maebot_laser_scan_t *msg)
        {
            assert(channel == "MAEBOT_LASER_SCAN");
            slam_->addScan(*msg);
        }

        void handleMotorFeedback(const lcm::ReceiveBuffer *rbuf,
                                 const std::string& channel,
                                 const maebot_motor_feedback_t *msg)
        {
            std::cout << "received feedback" << std::endl;

            assert(channel == "MAEBOT_MOTOR_FEEDBACK");
            slam_->lockSlamMutex();
            slam_->addMotorFeedback(*msg);
            if(slam_->scanReceived())
            {
                slam_->signal();
            }
            slam_->unlockSlamMutex();

            /*pthread_mutex_lock(&state_->loc.move_mutex);
           
            if (state_->need_init){
                state_->right_prev_dist = (state_->feedback.encoder_right_ticks/480.0) * 0.032 * 3.14;
                state_->left_prev_dist = (state_->feedback.encoder_left_ticks/480.0) * 0.032 * 3.14;
                state_->need_init = false;
            }
            
            double cur_dist = (state_->feedback.encoder_right_ticks/480.0) * 0.032 * 3.14;
            double right_step = cur_dist - state_->right_prev_dist;
            state_->right_prev_dist = cur_dist;
        
            cur_dist = (state_->feedback.encoder_left_ticks/480.0) * 0.032 *3.14;
            double left_step = cur_dist - state_->left_prev_dist;
            state_->left_prev_dist = cur_dist;
    
            //odometry
            double s_ = (right_step + left_step)/2;
            double delta_theta = (right_step - left_step)/0.08;
            double alpha = delta_theta/2;
            state_->loc.x += cos(state_->loc.theta + alpha) * s_; 
            state_->loc.y += sin(state_->loc.theta + alpha) * s_; 
            state_->loc.theta = eecs467::wrap_to_2pi(state_->loc.theta + delta_theta);
    
            maebot_motor_command_t cmd;
            cmd.utime = utime_now();
            
            if(reachGoal(state_)){
                std::cout << "reached ! " << state_->loc.x << " " 
                    << state_->loc.y << " " << state_->loc.theta << " "
                    << state_->next_x << " " << state_->next_y << std::endl;

                pthread_mutex_lock(&slam_->path_mutex_);
                if (slam_->bfs_result.empty()) {
                    pthread_mutex_unlock(&slam_->path_mutex_);
                    pthread_mutex_unlock(&state_->loc.move_mutex);
                    cmd.motor_right_speed = 0.0f;
                    cmd.motor_left_speed = 0.0f;
                    //state_->lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);
                    std::cout << "no pose" << std::endl;
                    return;
                }
    
                // get next goal location here
                state_->grid_mapper->toXYinMeters(slam_->bfs_result.back(), state_->next_x, state_->next_y);
                Point<double> gP(state_->grid_mapper->toX(slam_->bfs_result.back()),
                                 state_->grid_mapper->toY(slam_->bfs_result.back()));

                gP = yeecs467::grid_position_to_global_position(gP, state_->
                state_->next_x = state_->grid_mapper->toX(slam_->bfs_result.back());
                state_->next_y = state_->grid_mapper->toY(slam_->bfs_result.back());*/
                /*slam_->bfs_result.pop_back(); 
                pthread_mutex_unlock(&slam_->path_mutex_);
            }else{ 
                std::cout << "at" << state_->loc.x << " " 
                    << state_->loc.y << " " << state_->loc.theta << " move to "
                    << state_->next_x << " " << state_->next_y << std::endl;
            }
            pthread_mutex_unlock(&state_->loc.move_mutex);
    
            double angle;
            if(!rightAngle(state_, angle)){ 
                std::cout << "turning" << std::endl;
                angle = eecs467::angle_diff(angle, state_->loc.theta);
                if(angle > 0) {
                    cmd.motor_right_speed = 0.8*FORWARD_V;
                    cmd.motor_left_speed = 0;//0.8*FORWARD_V * -1;
                } else {
                    cmd.motor_right_speed = 0.8*FORWARD_V * -1;
                    cmd.motor_left_speed = 0;//0.8*FORWARD_V; 
                }
            } else { 
                cmd.motor_right_speed = FORWARD_V;
                cmd.motor_left_speed = FORWARD_V;
            }
    
            //state_->lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);

            //std::cout << "return" << std::endl;*/
        }
};

#endif
