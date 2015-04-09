#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <math.h>
#include <string>

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/matd.h"
#include "math/math_util.h"
#include "math/point.hpp"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_pose_t.hpp"
#include "Navigator.hpp"


#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f

class state_t
{
    private:
        pthread_t manager_thread;
        pthread_mutex_t manager_mutex;
        int running;
        lcm::LCM lcm;
        pthread_mutex_t lcm_mutex;

        maebot_pose_t maebot_locations[NUM_MAEBOT];
        pthread_mutex_t localization_mutex;

        std::deque<eecs467::Point<double> > maebot_dests[NUM_MAEBOT];
        std::deque<eecs467::Point<double> > maebot_paths[NUM_MAEBOT];

    public:
        state_t()
        {
            // initialize the things
            running = 1;
            pthread_mutex_init(&manager_mutex, NULL);
            pthread_mutex_init(&localization_mutex);

            // subscribe to lcm messages
            lcm.subscribe("UI_DEST_LIST", &state_t::dest_list_handler, this);
            lcm.subscribe("MAEBOT_LOCALIZATION_RED",   &state_t::maebot_localization_handler, this);
            lcm.subscribe("MAEBOT_LOCALIZATION_BLUE",  &state_t::maebot_localization_handler, this);
            lcm.subscribe("MAEBOT_LOCALIZATION_GREEN", &state_t::maebot_localization_handler, this);
        }

        ~state_t()
        {
            pthread_mutex_destroy(&manager_mutex);
            pthread_mutex_destroy(&localization_mutex);
        }

        void maebot_localization_handler(const lcm::ReceiveBuffer *rbuf, const std::string& channel, const maebot_pose_t *msg)
        {
            pthread_mutex_lock(&localization_mutex);
            if(channel == "MAEBOT_LOCALIZATION_RED")
            {
                maebot_locations[RED] = *msg;
            }
            else if(channel == "MAEBOT_LOCALIZATION_BLUE")
            {
                maebot_locations[BLUE] = *msg;
            }
            else if(channel == "MAEBOT_LOCALIZATION_GREEN")
            {
                maebot_locations[GREEN] = *msg;
            }
            else
            {
                // error
                std::cout << "tried to get localization data from wrong channel: " << channel << std::endl;
                exit(1);
            }
            pthread_mutex_unlock(&localization_mutex);
        }

        //void dest_list_handler(const lcm:ReceiveBuffer *rbuf, const std::string& channel, const ui_dest_list_t *msg)
};

// This thread continuously publishes command messages to the maebot
static void* send_cmds(void *data)
{
    state_t *state = (state_t *) data;
    uint32_t Hz = 20;

    while (state->running) {
        pthread_mutex_lock(&state->cmd_mutex);
        matd_t *click = matd_create_data(3, 1, state->last_click);
        double mag = matd_vec_mag(click);
        matd_t *n = click;
        if (mag != 0) {
            n = matd_vec_normalize(click);  // Leaks memory
        }
        double len = dmin(mag, state->joy_bounds);

        // Map vector direction to motor command.
        state->cmd.utime = utime_now();

        int sign_x = matd_get(n, 0, 0) >= 0; // > 0 if positive
        int sign_y = matd_get(n, 1, 0) >= 0; // > 0 if positive
        float magx = fabs(matd_get(n, 0, 0));
        float magy = fabs(matd_get(n, 1, 0));
        float x2y = magx > 0 ? (magx-magy)/magx : 0.0f;
        float y2x = magy > 0 ? (magy-magx)/magy : 0.0f;
        float scale = 1.0f*len/state->joy_bounds;

        // Quadrant check
        if (sign_y && sign_x) {
            // Quad I
            state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale*x2y;
            } else {
                state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale*y2x;
            }
        } else if (sign_y && !sign_x) {
            // Quad II
            state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale*x2y;
            } else {
                state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale*y2x;
            }
        } else if (!sign_y && !sign_x) {
            // Quad III
            state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_right_speed = MAX_FORWARD_SPEED*scale*x2y;
            } else {
                state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale*y2x;
            }
        } else {
            // Quad IV
            state->cmd.motor_right_speed = MAX_REVERSE_SPEED*scale;
            if (magx > magy) {
                state->cmd.motor_left_speed = MAX_FORWARD_SPEED*scale*x2y;
            } else {
                state->cmd.motor_left_speed = MAX_REVERSE_SPEED*scale*y2x;
            }
        }

        if (mag != 0) {
            matd_destroy(n);
        }
        matd_destroy(click);

        // Publish
        state->lcm->publish("MAEBOT_MOTOR_COMMAND", &(state->cmd));

        pthread_mutex_unlock(&state->cmd_mutex);
        
        auto a = state->grid_mapper.getOccupancyGrid().toLCM();
        state->lcm->publish("OCCUPANCY_GRID_GUI", &a);

        usleep(1000000/Hz);
    }

    return NULL;
}

static void* update_map(void *data)
{
    state_t *state = (state_t*) data;

    while(state->running)
    {
        state->grid_mapper.lockMapperMutex();
        while(state->grid_mapper.laserScansEmpty() || state->grid_mapper.posesEmpty())
        {
            state->grid_mapper.wait();
        }
        state->grid_mapper.unlockMapperMutex();
        std::cout << "received message" << std::endl;

        LaserScan updated_scan = state->grid_mapper.calculateLaserOrigins();
        if(!updated_scan.valid) { continue; }
        state->grid_mapper.updateGrid(updated_scan);
        state->grid_mapper.publishOccupancyGrid(updated_scan.end_pose);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    // === State initialization ============================
    state_t *state = new state_t;

    // === End =============================================

    // Clean up on Ctrl+C
    //signal(SIGINT, handler);

    getopt_add_bool(state->gopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(state->gopt, 'v', "verbose", 0, "Show extra debugging info");
    getopt_add_int(state->gopt, 'l', "limitKBs", "-1", "Remote display bandwith limit in KBs. < 0: unlimited.");
    getopt_add_int(state->gopt, 'p', "port", "15151", "Vx display port");

    if (!getopt_parse(state->gopt, argc, argv, 0) ||
        getopt_get_bool(state->gopt, "help"))
    {
        getopt_do_usage(state->gopt);
        exit(-1);
    }

    // Set up display
    verbose = getopt_get_bool(state->gopt, "verbose");

    // LCM subscriptions
    MaebotPoseHandler pose_handler(&state->grid_mapper);
    MaebotLaserScanHandler laser_scan_handler(&state->grid_mapper);

    state->lcm->subscribe("MAEBOT_POSE",
                          &MaebotPoseHandler::handleMessage,
                          &pose_handler);
    state->lcm->subscribe("MAEBOT_LASER_SCAN",
                          &MaebotLaserScanHandler::handleMessage,
                          &laser_scan_handler);
    std::cout << "listening" << std::endl;

    // Spin up thread(s)
    pthread_create(&state->cmd_thread, NULL, send_cmds, (void*)state);
//    pthread_create(&state->lcm_thread, NULL, receive_lcm, (void*)state);
    pthread_create(&state->update_map_thread, NULL, update_map, state);

    // Loop forever
    while(state->lcm->handle() == 0);

//    pthread_join (state->lcm_thread, NULL);
//    pthread_join (state->cmd_thread, NULL);

    return 0;
}
