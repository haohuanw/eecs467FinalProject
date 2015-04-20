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
#include <deque>

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"
#include "math/point.hpp"

#include "apps/eecs467_util.h"    // This is where a lot of the internals live

//lcm
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/bot_commands_t.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "math/angle_functions.hpp"
#include "math.h"
#include "PID.hpp"

using namespace std;

struct odometry_data
{
    int64_t timestamp;
    double delta_x;
    double delta_y;
    double delta_theta;
};

class state_t
{
    public:
        std::string color;
        lcm::LCM lcm_inst;
        PID pid;
        pthread_t lcm_thread;
        std::deque<odometry_data> odometry_updates;
    public:
        state_t(std::string color_in) : color(color_in), pid(&lcm_inst,color)
        {
            std::string init_str = "MAEBOT_PID_COMMAND_";
            std::string motor_feedback_channel = "MAEBOT_MOTOR_FEEDBACK_";
            init_str.append(color);
            motor_feedback_channel.append(color);
            lcm_inst.subscribe(init_str, &state_t::command_handler, this);
            lcm_inst.subscribe(motor_feedback_channel, &state_t::odometry_handler, this);
        }
        ~state_t() {}

        void command_handler(const lcm::ReceiveBuffer *rbuf, const string& channel, const bot_commands_t *msg)
        {
            //std::cout << "received command (" <<msg->x_rob<<","<<msg->y_rob<<")->("
            //            <<msg->x_dest<<","<<msg->y_dest<<")"<< std::endl;
            pthread_mutex_lock(&pid.command_mutex);
            std::cout << "command handler" << std::endl;
            
            //if same dest, error correction for time lag
            std::cout<<"Before pop:"<<odometry_updates.size()<<std::endl;
            while(!odometry_updates.empty() && odometry_updates.front().timestamp < msg->utime)
            {
                odometry_updates.pop_front();
            }
            std::cout<<"After pop:"<<odometry_updates.size()<<std::endl;
            pid.dest.x_rob = msg->x_rob;
            pid.dest.y_rob = msg->y_rob;
            pid.dest.theta_rob = msg->theta_rob;
            for(uint i = 0; i < odometry_updates.size(); i++)
            {
                pid.dest.x_rob += odometry_updates[i].delta_x;
                pid.dest.y_rob += odometry_updates[i].delta_y;
                pid.dest.theta_rob = eecs467::wrap_to_pi(pid.dest.theta_rob + odometry_updates[i].delta_theta);
            }

            if(pid.dest.x_rob == pid.dest.x_dest && pid.dest.y_rob == pid.dest.y_dest)
            {
                //std::cout << "received stop command" << std::endl;
                pid.stop_command = true;
                return;
            }
            else
            {
                pid.stop_command = false;
                pthread_cond_signal(&pid.command_cv);
            }

            
            if(pid.dest.x_dest != msg->x_dest || pid.dest.y_dest != msg->y_dest)
            {
                pid.dest.x_dest = msg->x_dest;
                pid.dest.y_dest = msg->y_dest;

                double dx = fabs(pid.dest.x_dest - pid.dest.x_rob);
                double dy = fabs(pid.dest.y_dest - pid.dest.y_rob);
                if((pid.dest.x_dest > pid.dest.x_rob) && (dy<dx)) 
                { 
                    pid.dest.theta_dest = 0; 
                }
                else if((pid.dest.x_dest < pid.dest.x_rob) && (dy<dx)) 
                { 
                    pid.dest.theta_dest = M_PI; 
                }
                else if((pid.dest.y_dest > pid.dest.y_rob) && (dx<dy)) 
                { 
                    pid.dest.theta_dest = M_PI/2.0; 
                }
                else if((pid.dest.y_dest < pid.dest.y_rob) && (dx<dy)){ 
                    pid.dest.theta_dest = -M_PI/2.0; 
                }
            }
            pthread_mutex_unlock(&pid.command_mutex);
        }

        void odometry_handler(const lcm::ReceiveBuffer *rbuf, const string& channel, const maebot_motor_feedback_t *msg)
        {

            if(pid.is_first_read)
            {
                pid.odometry.left_ticks = msg->encoder_left_ticks;
                pid.odometry.right_ticks = msg->encoder_right_ticks;
                pid.is_first_read = false;
                return;
            }

            double d_L = (msg->encoder_left_ticks - pid.odometry.left_ticks)*0.032*M_PI/480.0;
            double d_R = (msg->encoder_right_ticks - pid.odometry.right_ticks)*0.032*M_PI/480.0;
            double ticks_avg = double(d_L + d_R )/2.0;
            double d_theta = eecs467::wrap_to_pi(double(d_R - d_L)/0.08);
            double a = eecs467::wrap_to_pi(d_theta/2.0);
            pthread_mutex_lock(&pid.command_mutex);


            double deltax = cos(eecs467::wrap_to_pi(pid.dest.theta_rob + a))*ticks_avg;
            double deltay = sin(eecs467::wrap_to_pi(pid.dest.theta_rob + a))*ticks_avg;
        	odometry_updates.push_back(odometry_data{msg->utime, deltax, deltay, d_theta});
            std::cout<<"push back the odometry data"<<std::endl;
            pid.dest.x_rob = cos(eecs467::wrap_to_pi(pid.dest.theta_rob + a))*ticks_avg + pid.dest.x_rob;
            pid.dest.y_rob = sin(eecs467::wrap_to_pi(pid.dest.theta_rob + a))*ticks_avg + pid.dest.y_rob;
            pid.dest.theta_rob = eecs467::wrap_to_pi(d_theta + pid.dest.theta_rob);
            pid.odometry.left_ticks = msg->encoder_left_ticks;
            pid.odometry.right_ticks = msg->encoder_right_ticks;

            pthread_mutex_unlock(&pid.command_mutex);
        }
};

void *lcm_handle_thread(void *data)
{
    lcm::LCM *lcm = (lcm::LCM*) data;
    while(1)
    {
        lcm->handle();
    }
    return NULL;
}

int main(int argc, char* argv[])
{
    std::string color_in = std::string(argv[1]);
    state_t state(color_in);
    //std::cout<<"running bot driver on maebot color: "<<state.color<<std::endl;
    pthread_create(&state.lcm_thread, NULL, lcm_handle_thread, (void*)(&state.lcm_inst));
    while(1)
    {
        if(!state.pid.at_destination())
        {
            //std::cout<<"theta error:"<<state.pid.theta_error()<<std::endl;
            if(fabs(state.pid.theta_error()) > M_PI/12)
            {
                usleep(100000);
                //std::cout << "turn with theta error: " << state.pid.theta_error() << std::endl;
                state.pid.turn_to_dest();
            }
            else
            {
                //std::cout << "stright with theta error:" << state.pid.theta_error() << std::endl;
                state.pid.go_straight(color_in);
                //usleep(50000);
            }
        }
    }

    return 0;
}
