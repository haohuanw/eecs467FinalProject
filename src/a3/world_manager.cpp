#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <math.h>
#include <string>
#include <deque>

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/matd.h"
#include "math/math_util.h"
#include "math/point.hpp"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/ui_dest_list_t.hpp"
#include "lcmtypes/bot_commands_t.hpp"
#include "maebot_gui_t.hpp"
#include "Navigator.hpp"


#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f

class state_t
{
    public:
        pthread_t run_thread_red;
        pthread_t run_thread_blue;
        pthread_t run_thread_green;
        int running;
        lcm::LCM lcm;
        Navigator nav;

        maebot_pose_t maebot_locations[NUM_MAEBOT];
        pthread_mutex_t localization_mutex;

        std::deque<eecs467::Point<double> > maebot_dests[NUM_MAEBOT];
        pthread_mutex_t dests_mutexes[NUM_MAEBOT];
        pthread_cond_t dests_cvs[NUM_MAEBOT];
        
        std::deque<eecs467::Point<double> > maebot_paths[NUM_MAEBOT];
        bool reached_location[NUM_MAEBOT];
        pthread_mutex_t paths_mutexes[NUM_MAEBOT];
        pthread_cond_t paths_cvs[NUM_MAEBOT];
    public:
        state_t() : running(1), nav("../ground_truth/vmap.txt")
        {
            // initialize the things
            for(int i = 0; i < NUM_MAEBOT; i++)
            {
                maebot_locations[i].x = 0.0;
                maebot_locations[i].y = 0.0;
                maebot_locations[i].theta = 0;
                maebot_locations[i].utime = 0;
                reached_location[i] = false;

                pthread_mutex_init(&dests_mutexes[i], NULL);
                pthread_cond_init(&dests_cvs[i], NULL);
                pthread_mutex_init(&paths_mutexes[i], NULL);
                pthread_cond_init(&paths_cvs[i], NULL);
            }
            pthread_mutex_init(&localization_mutex, NULL);

            // subscribe to lcm messages
            lcm.subscribe("UI_DEST_LIST", &state_t::dest_list_handler, this);
            lcm.subscribe("MAEBOT_LOCALIZATION_RED",   &state_t::maebot_localization_handler, this);
            lcm.subscribe("MAEBOT_LOCALIZATION_BLUE",  &state_t::maebot_localization_handler, this);
            lcm.subscribe("MAEBOT_LOCALIZATION_GREEN", &state_t::maebot_localization_handler, this);
            lcm.subscribe("MAEBOT_PID_FEEDBACK_RED", &state_t::maebot_feedback_handler, this);
            lcm.subscribe("MAEBOT_PID_FEEDBACK_BLUE", &state_t::maebot_feedback_handler, this);
            lcm.subscribe("MAEBOT_PID_FEEDBACK_GREEN", &state_t::maebot_feedback_handler, this);
        }

        ~state_t()
        {
            pthread_mutex_destroy(&localization_mutex);
            for(int i = 0; i < NUM_MAEBOT; i++)
            {
                pthread_mutex_destroy(&dests_mutexes[i]);
                pthread_mutex_destroy(&paths_mutexes[i]);
                pthread_cond_destroy(&dests_cvs[i]);
                pthread_cond_destroy(&paths_cvs[i]);
            }
        }

        void maebot_localization_handler(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const maebot_pose_t *msg)
        {
            if(channel == "MAEBOT_LOCALIZATION_RED")
            {
                pthread_mutex_lock(&paths_mutexes[RED]);
                pthread_mutex_lock(&localization_mutex);
                
                maebot_locations[RED] = *msg;
                /*if(!maebot_paths[RED].empty())
                {
                    publish_to_maebot(RED, *msg, maebot_paths[RED].front());
                }*/
                
                pthread_mutex_unlock(&localization_mutex);
                pthread_mutex_unlock(&paths_mutexes[RED]);
            }
            else if(channel == "MAEBOT_LOCALIZATION_BLUE")
            {
                pthread_mutex_lock(&paths_mutexes[BLUE]);
                pthread_mutex_lock(&localization_mutex);
                
                maebot_locations[BLUE] = *msg;
                if(!maebot_paths[BLUE].empty())
                {
                    publish_to_maebot(BLUE, *msg, maebot_paths[BLUE].front());
                }
                
                pthread_mutex_unlock(&localization_mutex);
                pthread_mutex_unlock(&paths_mutexes[BLUE]);
            }
            else if(channel == "MAEBOT_LOCALIZATION_GREEN")
            {
                pthread_mutex_lock(&paths_mutexes[GREEN]);
                pthread_mutex_lock(&localization_mutex);
                
                maebot_locations[GREEN] = *msg;
                if(!maebot_paths[GREEN].empty())
                {
                    publish_to_maebot(GREEN, *msg, maebot_paths[GREEN].front());
                }
                
                pthread_mutex_unlock(&localization_mutex);
                pthread_mutex_unlock(&paths_mutexes[GREEN]);
            }
            else
            {
                // error
                std::cout << "tried to get localization data from wrong channel: " << channel << std::endl;
            }
        }

        void dest_list_handler(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const ui_dest_list_t *msg){
            std::cout << "received dest list from ui channel: " << channel << std::endl;
            if(msg->color == NONE)
            {
                return;
            }
            pthread_mutex_lock(&dests_mutexes[msg->color]);
            std::cout << "destinations: " << std::endl;
            for(int i = 0; i < msg->num_way_points; i++)
            {
                std::cout << "(" << msg->x_poses[i] << ", " << msg->y_poses[i] << ")\n";
                maebot_dests[msg->color].push_back(eecs467::Point<double>{msg->x_poses[i], msg->y_poses[i]});
            }
            pthread_cond_signal(&dests_cvs[msg->color]);
            pthread_mutex_unlock(&dests_mutexes[msg->color]);
        }

        void maebot_feedback_handler(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const bot_commands_t *msg)
        {
            if(channel == "MAEBOT_PID_FEEDBACK_RED")
            {
                pthread_mutex_lock(&paths_mutexes[RED]);
                std::cout << "red maebot reached location" << std::endl;
                reached_location[RED] = true;
                pthread_cond_signal(&paths_cvs[RED]);
                pthread_mutex_unlock(&paths_mutexes[RED]);
            }
            else if(channel == "MAEBOT_PID_FEEDBACK_BLUE")
            {
                pthread_mutex_lock(&paths_mutexes[BLUE]);
                std::cout << "blue maebot reached location" << std::endl;
                reached_location[BLUE] = true;
                pthread_cond_signal(&paths_cvs[BLUE]);
                pthread_mutex_unlock(&paths_mutexes[BLUE]);
            }
            else if(channel == "MAEBOT_PID_FEEDBACK_GREEN")
            {
                pthread_mutex_lock(&paths_mutexes[GREEN]);
                std::cout << "green maebot reached location" << std::endl;
                reached_location[GREEN] = true;
                pthread_cond_signal(&paths_cvs[GREEN]);
                pthread_mutex_unlock(&paths_mutexes[GREEN]);
            }
            else
            {
                // error
                std::cout << "skip feedback message channel: " << channel << std::endl;
            }
        }

        void publish_to_maebot(maebot_color maebot, maebot_pose_t location, eecs467::Point<double> dest)
        {
            std::cout << "color: " << maebot << std::endl;
            std::cout << "Publishing location: (" << location.x << ", " << location.y << ") -> (" << dest.x << ", " << dest.y << ")\n";
            bot_commands_t cmd;
            cmd.x_rob = location.x;
            cmd.y_rob = location.y;
            cmd.theta_rob = location.theta;
            cmd.x_dest = dest.x;
            cmd.y_dest = dest.y;
            if(maebot == RED)
            {
                lcm.publish("MAEBOT_PID_COMMAND_RED", &cmd);
                std::cout<<"Publish to RED maebot"<<std::endl;
            }
            else if(maebot == BLUE)
            {
                lcm.publish("MAEBOT_PID_COMMAND_BLUE", &cmd);
            }
            else if(maebot == GREEN)
            {
                lcm.publish("MAEBOT_PID_COMMAND_GREEN", &cmd);
            }
            else
            {
                std::cout << "error: wrong maebot color passed in to publish_to_maebot function!!\n";
                exit(1);
            }
        }

        void publish_to_ui(maebot_color maebot)
        {
            std::cout << "publishing to ui" << std::endl;
            ui_dest_list_t data;
            data.num_way_points = 1;
            data.color = (int) maebot;
            data.x_poses.push_back(maebot_dests[maebot].front().x);
            data.y_poses.push_back(maebot_dests[maebot].front().y);
            lcm.publish("MAEBOT_DEST", &data);
        }

        bool within_error(maebot_color maebot)
        {
            return fabs(maebot_locations[maebot].x - maebot_paths[maebot].front().x) < 0.1 &&
                   fabs(maebot_locations[maebot].y - maebot_paths[maebot].front().y) < 0.1;
        }
};


state_t *state;
static void* run_thread(void *data)
{
    maebot_color *color = (maebot_color *) data;
    uint32_t Hz = 20;

    while (state->running)
    {
        pthread_mutex_lock(&state->dests_mutexes[*color]);
        std::cout << "locked dests_mutex before checking dests.empty()" << std::endl;
        while(state->maebot_dests[*color].empty())
        {
            pthread_cond_wait(&state->dests_cvs[*color], &state->dests_mutexes[*color]);
        }
        pthread_mutex_unlock(&state->dests_mutexes[*color]);
        std::cout << "exited first wait" << std::endl;

        // if no path (not calculated yet), create path
        pthread_mutex_lock(&state->paths_mutexes[*color]);
        std::cout << "locked paths mutex" << std::endl;
        if(state->maebot_paths[*color].empty())
        {
            std::cout << "creating path to first dest" << std::endl;
            pthread_mutex_lock(&state->localization_mutex);
            std::cout << "locked localization mutex when path empty" << std::endl;
            pthread_mutex_lock(&state->dests_mutexes[*color]);
            std::cout << "locked dests mutex when path empty" << std::endl;
            
            state->maebot_paths[*color] = state->nav.pathPlan(eecs467::Point<double>{state->maebot_locations[*color].x,
                                                              state->maebot_locations[*color].y},
                                                              state->maebot_dests[*color].front());
            std::cout << "Calculated path: " << std::endl;
            for(uint i = 0; i < state->maebot_paths[*color].size(); i++)
            {
                std::cout << "(" << state->maebot_paths[*color][i].x << ", "
                          << state->maebot_paths[*color][i].y << ") -> ";
            }
            std::cout << std::endl;
            state->publish_to_maebot(*color, state->maebot_locations[*color], state->maebot_paths[*color].front());
            state->publish_to_ui(*color);
            
            pthread_mutex_unlock(&state->dests_mutexes[*color]);
            pthread_mutex_unlock(&state->localization_mutex);
        }
        pthread_mutex_unlock(&state->paths_mutexes[*color]);
        std::cout << "unlocked mutexes" << std::endl;

        // if path, check if reached next point in path

        // wait until maebot thinks it has reached it's next point in the path
        pthread_mutex_lock(&state->paths_mutexes[*color]);
        std::cout << "locked paths mutex before wait till maebot done" << std::endl;
        while(!state->reached_location[*color])
        {
            pthread_cond_wait(&state->paths_cvs[*color], &state->paths_mutexes[*color]);
        }
        pthread_mutex_unlock(&state->paths_mutexes[*color]);
        std::cout << "maebot reached location" << std::endl;

        // check if maebot actually reached location:
        
        pthread_mutex_lock(&state->paths_mutexes[*color]);
        std::cout << "locked paths mutex before checking if maebot reached location" << std::endl;
        pthread_mutex_lock(&state->localization_mutex);
        std::cout << "locked localization mutex before checking if maebot reached location" << std::endl;
        if(!state->within_error(*color))
        {
            std::cout << "republishing destination" << std::endl;
            state->publish_to_maebot(*color, state->maebot_locations[*color], state->maebot_paths[*color].front());
        }
        else
        {
            std::cout << "publishing next destination" << std::endl;
            state->maebot_paths[*color].pop_front();
            if(state->maebot_paths[*color].empty())
            {
                std::cout << "popping dest off deque" << std::endl;
                pthread_mutex_lock(&state->dests_mutexes[*color]);
                std::cout << "grabbed dests mutex when path empty" << std::endl;
                state->maebot_dests[*color].pop_front();
                pthread_mutex_unlock(&state->dests_mutexes[*color]);
                pthread_mutex_unlock(&state->localization_mutex);
                pthread_mutex_unlock(&state->paths_mutexes[*color]);
                continue;
            }
            state->publish_to_maebot(*color, state->maebot_locations[*color], state->maebot_paths[*color].front());
        }
        pthread_mutex_unlock(&state->localization_mutex);
        pthread_mutex_unlock(&state->paths_mutexes[*color]);

        usleep(1000000/Hz);
    }

    return NULL;
}
int main(int argc, char **argv)
{
    // === State initialization ============================
    state = new state_t;

    // Spin up thread(s)
    pthread_create(&state->run_thread_red, NULL, run_thread, (void*)new maebot_color(RED));
    pthread_create(&state->run_thread_blue, NULL, run_thread, (void*)new maebot_color(BLUE));
    pthread_create(&state->run_thread_green, NULL, run_thread, (void*)new maebot_color(GREEN));

    // Loop forever
    while(state->lcm.handle() == 0);

    return 0;
}
