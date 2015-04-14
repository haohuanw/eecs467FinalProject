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
class state_t
{
    public:
        lcm::LCM lcm_inst;
        PID pid;
        pthread_t lcm_thread;

        state_t() : pid(&lcm_inst)
        {
            lcm_inst.subscribe("MAEBOT_PID_COMMAND_RED", &state_t::command_handler, this);
            lcm_inst.subscribe("MAEBOT_MOTOR_FEEDBACK", &state_t::odometry_handler, this);
        }
        ~state_t() {}

        void command_handler(const lcm::ReceiveBuffer *rbuf, const string& channel, const bot_commands_t *msg)
        {
            pthread_mutex_lock(&pid.command_mutex);

            pid.dest.x_rob = msg->x_rob;
            pid.dest.y_rob = msg->y_rob;
            pid.dest.theta_rob = msg->theta_rob;

            pid.dest.x_dest = msg->x_dest;
            pid.dest.y_dest = msg->y_dest;

            if(pid.dest.x_rob == pid.dest.x_dest && pid.dest.y_rob == pid.dest.y_dest)
            {
                pid.stop_command = true;
            }
            else
            {
                pid.stop_command = false;
                pthread_cond_signal(&pid.command_cv);
            }

            if(pid.dest.x_dest > pid.dest.x_rob) { pid.dest.theta_dest = 0; }
            else if(pid.dest.x_dest < pid.dest.x_rob) { pid.dest.theta_dest = M_PI; }
            else if(pid.dest.y_dest > pid.dest.y_rob) { pid.dest.theta_dest = M_PI/2.0; }
            else { pid.dest.theta_dest = -M_PI/2.0; }

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
            pid.dest.x_rob = cos(eecs467::wrap_to_pi(pid.dest.theta_rob + a))*ticks_avg + pid.dest.x_rob;
            pid.dest.y_rob = sin(eecs467::wrap_to_pi(pid.dest.theta_rob + a))*ticks_avg + pid.dest.y_rob;
            pid.dest.theta_rob = eecs467::wrap_to_pi(d_theta + pid.dest.theta_rob);
            pid.odometry.left_ticks = msg->encoder_left_ticks;
            pid.odometry.right_ticks = msg->encoder_right_ticks;
            /*cout << "x_rob: " << pid.dest.x_rob 
                 << " y_rob: " << pid.dest.y_rob 
                 << " theta_rob: " << pid.dest.theta_rob << endl;*/
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

int main()
{
    state_t state;
    pthread_create(&state.lcm_thread, NULL, lcm_handle_thread, (void*)(&state.lcm_inst));
    while(1)
    {
        if(!state.pid.at_destination())
        {
            if(state.pid.theta_error() > M_PI/6)
            {
                std::cout << "theta error: " << state.pid.theta_error() << std::endl;
                std::cout << "turn" << std::endl;
                state.pid.turn_to_dest();
            }
            else
            {
                std::cout << "stright" << std::endl;
                state.pid.go_straight();
            }
        }
    }

    return 0;
}
