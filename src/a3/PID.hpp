#ifndef _PID_HPP_
#define _PID_HPP_

/*prototype for controlling robot motion*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>

// common
#include "math/point.hpp"

//lcm
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/bot_commands_t.hpp"
#include "math/angle_functions.hpp"
#include "math.h"

#define START_SPEED 0.20f
#define TURN_START_SPEED 0.16f
#define MIN_SPEED 0.18f
#define MAX_SPEED 0.22f
#define MIN_GAIN 0.0004f
#define MAX_GAIN 0.0008f

struct odo_data
{
    int32_t left_ticks;
    int32_t right_ticks;
};

struct move_data
{
    // robot pose
    double x_rob;
    double y_rob;
    double theta_rob;

    // destination pose
    double x_dest;
    double y_dest;
    double theta_dest;
};

class PID
{
    public:
        move_data dest;
        odo_data odometry;
        bool is_first_read;
        bool stop_command;
        bool turning;
        lcm::LCM *lcm;
        std::string color;
        std::string feedback_channel;
        pthread_mutex_t command_mutex;
        pthread_cond_t command_cv;

    public:
        PID(lcm::LCM *lcm_,std::string c) : is_first_read(true), stop_command(false), turning(false), lcm(lcm_)
    {
        pthread_mutex_init(&command_mutex, NULL);
        pthread_cond_init(&command_cv, NULL);
        color = c;
        dest.x_rob = 0;
        dest.y_rob = 0;
        dest.theta_rob = 0;
        dest.x_dest = 0;
        dest.y_dest = 0;
        dest.theta_dest = 0;
        feedback_channel = "MAEBOT_PID_FEEDBACK_";
        feedback_channel.append(color);
        odometry.left_ticks = 0;
        odometry.right_ticks = 0;
    }

        ~PID()
        {
            pthread_mutex_destroy(&command_mutex);
        }

        double dist(double x1, double y1, double x2, double y2)
        {
            return sqrt(((y2-y1)*(y2-y1)) + ((x2-x1)*(x2-x1)));
        }

        double theta_error()
        {
            pthread_mutex_lock(&command_mutex);
            double retval = fabs(eecs467::angle_diff(dest.theta_rob, dest.theta_dest));
            pthread_mutex_unlock(&command_mutex);
            return retval;
        }

        bool at_destination()
        {
            double path_pos, path_dest, lane_pos, lane_dest;

            pthread_mutex_lock(&command_mutex);
            double theta_dest = dest.theta_dest;
            get_path_and_lane(path_pos, lane_pos, path_dest, lane_dest);
            pthread_mutex_unlock(&command_mutex);

            return !not_done(path_pos, path_dest, theta_dest);
        }

        bool not_done(double& path_pos, double& path_dest, double& theta_dest)
        {
            if(theta_dest == 0 || theta_dest == M_PI/2.0)
            {
                if(path_pos < path_dest) return true;
            }
            else
            {
                if(path_dest < path_pos) return true;
            }
            return false;
        }

        int too_far(double theta_dest, double lane_pos, double lane_dest, double threshold)
        {
            if(theta_dest == 0 || -theta_dest == M_PI/2.0)
            {
                if(lane_pos - lane_dest > threshold)
                    return 1;
                else if(lane_pos - lane_dest < -threshold)
                    return -1;
                else return 0;
            }
            else
            {
                if(lane_dest - lane_pos > threshold)
                    return 1;
                else if(lane_dest - lane_pos < -threshold)
                    return -1;
                else return 0;
            }
        }

        double lane_delta(double lane_pos, double lane_dest, double theta_dest){

            double width = 0.15;

            if(theta_dest == 0){
                return (lane_pos - lane_dest)/width;
            }
            else if(theta_dest == M_PI/2){
                return (lane_dest - lane_pos)/width;
            }
            else if(theta_dest == M_PI || theta_dest == -M_PI){
                return (lane_dest - lane_pos)/width;
            }
            else{
                return (lane_pos - lane_dest)/width;
            }

        }

        // if get a stop command, wait until get a move command
        void wait_until_clear(maebot_motor_command_t cmd)
        {
            cmd.motor_left_speed = 0.0;
            cmd.motor_right_speed = 0.0;
            lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);
            pthread_mutex_lock(&command_mutex);
            while(stop_command)
            {
                pthread_cond_wait(&command_cv, &command_mutex);
            }
            pthread_mutex_unlock(&command_mutex);
        }

        void turn_to_dest()
        {
            //std::cout<<"turn_to_dest current theta:"<<dest.theta_rob<<"  dest theta:"<<dest.theta_dest<<std::endl;
            maebot_motor_command_t cmd;
            bot_commands_t ret_msg = {0, 0, 0, 0, 0, 0, 1};
            double speed = 0.064;

            pthread_mutex_lock(&command_mutex);
            turning = true;
            double angle_rob = dest.theta_rob;
            double angle_dest = dest.theta_dest;
            double init_diff = fabs(eecs467::angle_diff(angle_rob,angle_dest));
            double curr_diff = fabs(eecs467::angle_diff(angle_rob,angle_dest));
            pthread_mutex_unlock(&command_mutex);



            //double angle_diff = eecs467::angle_diff(angle_dest, angle_rob);
            //std::cout<<"turn_to_dest angle_diff: "<<angle_diff<<std::endl;
            while(init_diff-curr_diff < 0.9 * init_diff)    
            //while(fabs(angle_diff) > M_PI/60)
            {
                if(stop_command) { wait_until_clear(cmd); }
                    cmd.motor_left_speed =  -speed*(curr_diff/init_diff)- 0.13;
                    cmd.motor_right_speed = speed*(curr_diff/init_diff) + 0.13;
                    if(eecs467::angle_diff(angle_rob, angle_dest) > 0)
                    {
                        cmd.motor_left_speed = cmd.motor_left_speed;
                        cmd.motor_right_speed = cmd.motor_right_speed;
                    }
                    else
                    {
                        cmd.motor_left_speed = -cmd.motor_left_speed;
                        cmd.motor_right_speed = -cmd.motor_right_speed;
                    }



                lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);
                usleep(5000);

                pthread_mutex_lock(&command_mutex);
                double angle_rob = dest.theta_rob;
                double angle_dest = dest.theta_dest;
                curr_diff = fabs(eecs467::angle_diff(angle_rob,angle_dest));
                pthread_mutex_unlock(&command_mutex);
                //angle_diff = eecs467::angle_diff(angle_dest, angle_rob);
            }
            pthread_mutex_lock(&command_mutex);
            turning = false;
            pthread_mutex_unlock(&command_mutex);

            cmd.motor_left_speed = 0.0;
            cmd.motor_right_speed = 0.0;
            lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);

            lcm->publish(feedback_channel, &ret_msg);
        }

        void get_path_and_lane(double& path_pos, double& lane_pos, double& path_dest, double& lane_dest)
        {
            if(dest.theta_dest == 0 || dest.theta_dest == M_PI)
            {
                path_pos = dest.x_rob;
                lane_pos = dest.y_rob;
                path_dest = dest.x_dest;
                lane_dest = dest.y_dest;
            }
            else // if dest.theta_dest == -M_PI/2.0 || dest.theta_dest == M_PI/2.0
            {
                path_pos = dest.y_rob;
                lane_pos = dest.x_rob;
                path_dest = dest.y_dest;
                lane_dest = dest.x_dest;
            }
        }

        void correct_motor_speeds(maebot_motor_command_t& cmd, double& path_pos, double& lane_pos, double& path_dest, double& lane_dest, double& theta_rob, double& theta_dest, std::string  color)
        {
            double left_speed, right_speed;
            bool r = 0;
            bool g = 0;
            bool b = 0;
            double basel;
            double baser;
            double T_gain;
            double S_gain;
             if( color == "RED"){
                r = 1;
             }
             else if(color == "GREEN"){
                g = 1;
             }
             else{
                b = 1;
             }

            if(r ==1 ){
                basel = 0.19; 
                baser = 0.19;
                T_gain = 0.05;
                S_gain = 0.1;
            }
            else if(g == 1){
                basel = 0.18;
                baser = 0.155;
                T_gain = 0.05;
                S_gain = 0.1;
            }
            else{
                basel = 0.19;
                baser = 0.155;
                T_gain = 0.05;
                S_gain = 0.1;
            }
            //std::cout << "theta robot: " << theta_rob << std::endl;
            //std::cout << "theta dest:  " << theta_dest << std::endl;
            // if on left side of lane, move right
            /*if(too_far(theta_dest, lane_pos, lane_dest, 0.04) == 1 &&
              fabs(eecs467::wrap_to_pi(theta_rob - theta_dest) < M_PI/40))
              {
              std::cout << "Far right" << std::endl;
              if(cmd.motor_left_speed > MIN_SPEED)
              cmd.motor_left_speed -= MIN_GAIN;
              else if(cmd.motor_right_speed < MAX_SPEED)
              cmd.motor_right_speed += MIN_GAIN;
              }

            // if on right side of lane, move left
            else if(too_far(theta_dest, lane_pos, lane_dest, 0.04) == -1 &&
            fabs(eecs467::wrap_to_pi(theta_rob - theta_dest) < M_PI/40))
            {
            std::cout << "Far left" << std::endl;
            if(cmd.motor_left_speed < MAX_SPEED)
            cmd.motor_left_speed += MIN_GAIN;
            else if(cmd.motor_right_speed > MIN_SPEED)
            cmd.motor_right_speed -= MIN_GAIN;
            }

            // if feering left, move right
            else if(eecs467::wrap_to_pi(theta_rob - theta_dest) > M_PI/40)
            {
            std::cout << "veering left" << std::endl;
            if(cmd.motor_left_speed < MAX_SPEED)
            cmd.motor_left_speed += MAX_GAIN;
            else if(cmd.motor_right_speed > MIN_SPEED)
            cmd.motor_right_speed -= MAX_GAIN;
            }

            // if veering right, move left
            else if(eecs467::wrap_to_pi(theta_rob - theta_dest) < -M_PI/40)
            {
            std::cout << "veering right" << std::endl;
            if(cmd.motor_left_speed > MIN_SPEED)
            cmd.motor_left_speed -= MAX_GAIN;
            else if(cmd.motor_right_speed < MAX_SPEED)
            {
            cmd.motor_right_speed += MAX_GAIN;
            }
            }
            else{

            cmd.motor_left_speed = START_SPEED;
            cmd.motor_right_speed = START_SPEED;

            if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == 1 ){
            std::cout << "straight left" << std::endl;
            cmd.motor_left_speed = START_SPEED;
            cmd.motor_right_speed = START_SPEED + 0.005;
            }
            //IF WE'RE ON THE RIGHT SIDE OF THE LANE, CORRECT BY MOVING LEFT
            else if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == -1 ) {
            std::cout << "straight right" << std::endl;
            cmd.motor_left_speed = START_SPEED + 0.005;
            cmd.motor_right_speed = START_SPEED;
            }
            }*/
            //std::cout << "updated cmd: " << cmd.motor_left_speed << "  " << cmd.motor_right_speed << std::endl;

            if( too_far(theta_dest, lane_pos, lane_dest, 0.04) != 0 && abs(180*eecs467::angle_diff(theta_rob, theta_dest)/M_PI) < 8 ){
                //std::cout << "too far" << std::endl;
                if(r == 1){
                    T_gain = 0.06; //green mabebot 0.05
                    S_gain = 0.1; 
                }
                else if(g == 1){
                    T_gain = 0.05;
                    S_gain = 0.1; 
                }
                 else if(b == 1){
                    T_gain = 0.07;
                    S_gain = 0.12; 
                }
            }

            else if(  abs(180*eecs467::angle_diff(theta_rob, theta_dest)/M_PI) > 4.0 ){
                if(r==1){
                    T_gain = 0.1; //green mabebot 0.08
                    S_gain = 0.04;  //green mabebot 0.06
                }
                else if(g==1){
                    T_gain = 0.08; 
                    S_gain = 0.06;
                }
                 else if(b == 1){
                    T_gain = 0.1;
                    S_gain = 0.06; 
                }
                //std::cout <<"too wide" << std::endl; 

            }
            else{
                if(r == 1){
                    T_gain = 0.08; //green mabebot 0.06
                    S_gain = 0.08;  //green mabebot 0.06
                }
                 else if(g==1){
                    T_gain = 0.06; 
                    S_gain = 0.06;
                }
                else{
                    T_gain = 0.06; 
                    S_gain = 0.06;
                }
            }

            left_speed = basel + T_gain*(180*eecs467::angle_diff(theta_rob, theta_dest)/M_PI)/180.0 + S_gain*lane_delta(lane_pos, lane_dest, theta_dest); 
            right_speed = baser - T_gain*(180*eecs467::angle_diff(theta_rob, theta_dest)/M_PI)/180.0 - S_gain*lane_delta(lane_pos, lane_dest, theta_dest);


            if(left_speed > basel + 0.02){
                left_speed = basel + 0.02;
            }
            if(right_speed > baser + 0.02){
                right_speed =baser + 0.02;
            }
            if(left_speed < basel - 0.02){
                left_speed = basel - 0.02;
            }
            if(right_speed < baser -0.02){
                right_speed = baser - 0.02;
            }

            cmd.motor_left_speed = left_speed;
            cmd.motor_right_speed = right_speed;

        }

        void go_straight(std::string color)
        {
            //std::cout << "inside go straight" << std::endl;
            //std::cout << "current position: " << dest.x_rob << " " << dest.y_rob << std::endl;
            //std::cout << "dest position:    " << dest.x_dest << " " << dest.y_dest << std::endl;
            maebot_motor_command_t cmd;
            bot_commands_t bot_cmd = {0, 0, 0, 0, 0, 0, 1};
            cmd.motor_left_speed = START_SPEED;
            cmd.motor_right_speed = START_SPEED;
            double path_pos, path_dest, lane_pos, lane_dest;

            pthread_mutex_lock(&command_mutex);
            double theta_rob = dest.theta_rob;
            double theta_dest = dest.theta_dest;

            get_path_and_lane(path_pos, lane_pos, path_dest, lane_dest);
            pthread_mutex_unlock(&command_mutex);
            //std::cout << "after first update" << std::endl;

            while(not_done(path_pos, path_dest, theta_dest))
            {
                if(stop_command) { wait_until_clear(cmd); }
                //std::cout << "current position: " << dest.x_rob << " " << dest.y_rob << std::endl;
                //std::cout << "dest position:    " << dest.x_dest << " " << dest.y_dest << std::endl;
                //std::cout << "current: (" << dest.x_rob << ", " << dest.y_rob << ", " << dest.theta_rob << ")\n";
                //std::cout << "dest:    (" << dest.x_dest << ", " << dest.y_dest << ", " << dest.theta_dest << ")\n";
                //std::cout<<"current theta:"<<theta_rob<<"  dest theta:"<<theta_dest<<std::endl;
                correct_motor_speeds(cmd, path_pos, lane_pos, path_dest, lane_dest, theta_rob, theta_dest, color);

                lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);
                //std::cout << "published motor command " << cmd.motor_left_speed << "  " << cmd.motor_right_speed << std::endl;
                usleep(5000);

                pthread_mutex_lock(&command_mutex);
                theta_rob = dest.theta_rob;
                theta_dest = dest.theta_dest;
                get_path_and_lane(path_pos, lane_pos, path_dest, lane_dest);
                pthread_mutex_unlock(&command_mutex);
                //std::cout << "after second update" << std::endl;
            }
            cmd.motor_left_speed = 0.0;
            cmd.motor_right_speed = 0.0;
            lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);
            lcm->publish(feedback_channel, &bot_cmd);
            //std::cout << "outside while loop" << std::endl;
        }
};

#endif
