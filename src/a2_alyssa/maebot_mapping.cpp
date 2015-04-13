#include <stdio.h>
#include <fenv.h>
#include <iostream>
#include <fstream>
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
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "vx/vx.h"
#include "vx/vxo_drawables.h"
#include "vx/vx_remote_display_source.h"

#include "maebot_handlers.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "OccupancyGridMapper.hpp"
#include "Slam.hpp"
#include "MagicNumbers.hpp"

#define MAX_REVERSE_SPEED -1.0f
#define MAX_FORWARD_SPEED 1.0f

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

typedef struct state state_t;
struct state
{
    int running;

    double joy_bounds;
    double last_click[3];

    maebot_motor_command_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;

    //pthread_t lcm_thread;
    pthread_t render_thread;
    pthread_mutex_t render_mutex;
    pthread_mutex_t layer_mutex;

    pthread_t update_map_thread;

    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;

    lcm::LCM *lcm;
    pthread_mutex_t lcm_mutex;

    OccupancyGridMapper *grid_mapper;
    Slam *slam;
};

static int verbose = 0;

static state_t *global_state;

//static void * receive_lcm(void *data) 
//{
//    state_t *state = (state_t *)data; 
//    
//    while(1)
//        state->lcm->handle();
//    return NULL;
//}

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
        if(task2)
        {
            auto a = state->grid_mapper->getOccupancyGrid().toLCM();
            state->lcm->publish("OCCUPANCY_GRID_GUI_TASK2", &a);
        }

        usleep(1000000/Hz);
    }

    return NULL;
}

static void* update_map(void *data)
{
    state_t *state = (state_t*) data;

    while(state->running)
    {
        if(task2)
        {
            state->slam->lockSlamMutex();
            while(!state->slam->scanReceived())
            {
                state->slam->wait();
            }
            state->slam->unlockSlamMutex();

            state->slam->updateParticles();
            //state->slam->publish();
        }
        else if(task1)
        {
            state->grid_mapper->lockMapperMutex();

            while(state->grid_mapper->laserScansEmpty() || state->grid_mapper->posesEmpty())        
                state->grid_mapper->wait();
            state->grid_mapper->unlockMapperMutex();
            std::cout << "received message" << std::endl;

            LaserScan updated_scan = state->grid_mapper->calculateLaserOrigins();
            if(!updated_scan.valid) continue;

            state->grid_mapper->updateGrid(updated_scan);
            state->grid_mapper->publishOccupancyGrid(updated_scan.end_pose);
        }
    }
    return NULL;
}

int main(int argc, char **argv)
{
    // === State initialization ============================
    state_t *state = new state_t;
    global_state = state;
    state->gopt = getopt_create();
    state->last_click[0] = 0;
    state->last_click[1] = 0;
    state->last_click[2] = 0;
    state->joy_bounds = 10.0;

    state->running = 1;
    state->lcm = new lcm::LCM;
    pthread_mutex_init(&state->layer_mutex, NULL);
    pthread_mutex_init(&state->cmd_mutex, NULL);
    pthread_mutex_init(&state->lcm_mutex, NULL);
    pthread_mutex_init(&state->render_mutex, NULL);

//feenableexcept(FE_DIVBYZERO| FE_INVALID|FE_OVERFLOW); 
feenableexcept(FE_ALL_EXCEPT & ~FE_INEXACT & ~FE_UNDERFLOW);

    if(task2)
    {
        std::cout << "making grid" << std::endl;
        std::ifstream input("../src/a1_task2/figure_eight.txt");
        assert(input.good());
        int width = 0, 
            height = 0;
        double metersPerCell = 0, 
               logOdds = 0;
        input >> width >> height >> metersPerCell;
        assert(width && height && metersPerCell);
        state->grid_mapper = new OccupancyGridMapper(width*metersPerCell, height*metersPerCell, metersPerCell);
        for(int y = 0; y < height; y++)
        {
            for(int x = 0; x < width; x++)
            {
                input >> logOdds;
                state->grid_mapper->setLogOddsMapper(x, y, logOdds);
            }
        }
    }
    else
    {
        state->grid_mapper = new OccupancyGridMapper;
    }
    state->slam = new Slam(state->grid_mapper);
    state->grid_mapper->setLCM(state->lcm);
    state->slam->setLCM(state->lcm);
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
    MaebotLCMHandler lcm_handler(state->grid_mapper, state->slam);
    state->lcm->subscribe("MAEBOT_LASER_SCAN",
            &MaebotLCMHandler::handleLaserScan,
            &lcm_handler);

    state->lcm->subscribe("MAEBOT_POSE",
            &MaebotLCMHandler::handlePose,
            &lcm_handler);

    state->lcm->subscribe("MAEBOT_MOTOR_FEEDBACK",
            &MaebotLCMHandler::handleMotorFeedback,
            &lcm_handler);

    std::cout << "listening" << std::endl;

    // Spin up threads
    pthread_create(&state->cmd_thread, NULL, send_cmds, (void*)state);
    //    pthread_create(&state->lcm_thread, NULL, receive_lcm, (void*)state);
    pthread_create(&state->update_map_thread, NULL, update_map, state);

    // Loop forever
    while(state->lcm->handle() == 0);

    pthread_join (state->update_map_thread, NULL);
    pthread_join (state->cmd_thread, NULL);

    return 0;
}
