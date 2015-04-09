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
#include <time.h>

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
#include "MagicNumbers.hpp"
#include "state.hpp"


#define MAX_REVERSE_SPEED 0.125f
#define MAX_FORWARD_SPEED 0.2f

#define dmax(A,B) A < B ? B : A
#define dmin(A,B) A < B ? A : B

static int verbose = 0;

static state_t *global_state;

static void rotateTowards(double dir, state_t *state)
{
    dir = (dir > 0 ? 1 : (dir < 0 ? -1 : 0));

    //PUBLISH TO LCM!
    state->cmd.motor_left_speed = -dir*MAX_FORWARD_SPEED;
    state->cmd.motor_right_speed = dir*MAX_FORWARD_SPEED;
    state->lcm->publish("MAEBOT_MOTOR_COMMAND", &(state->cmd));
}

static void moveForward(state_t *state)
{
    //PUBLISH TO LCM!
    state->cmd.motor_left_speed = MAX_FORWARD_SPEED;
    state->cmd.motor_right_speed = MAX_FORWARD_SPEED;
    state->lcm->publish("MAEBOT_MOTOR_COMMAND", &(state->cmd));
}

/*static void moveTowardsPoint(maebot_pose_t a, int nextCell, state_t *state)
{
    if(state->slam->bfs_result.size() <= 0)
        return;

    double theta = eecs467::angle_diff(atan2(state->grid_mapper->toY(nextCell) - a.y, state->grid_mapper->toX(nextCell) - a.x), a.theta);
    if(abs(theta) > THETA_VARIANCE_MAX)
    {
        std::cout << "rotating towards: " << theta << std::endl;
        rotateTowards(theta, state);
    }
    else
    {
        std::cout << "moving forward" << std::endl;
        moveForward(state);
        auto d2 = (a.x - state->grid_mapper->toX(nextCell)) * (a.x - state->grid_mapper->toX(nextCell)) + (a.y - state->grid_mapper->toY(nextCell)) * (a.y - state->grid_mapper->toY(nextCell));
        if(d2 <= WAYPOINT_RADIUS * WAYPOINT_RADIUS)
        { 
            pthread_mutex_lock(&state->slam->path_mutex_);
            state->slam->bfs_result.pop_back();
            pthread_mutex_unlock(&state->slam->path_mutex_);
            std::cout << "REACHED WAYPOINT. bfs_result.size() = " << state->slam->bfs_result.size() << std::endl;
        }
    }
}*/

/*bool reachGoal(state_t *state) {
    return (abs(state->loc.x - state->next_x) < SAME_REGION_R && 
            abs(state->loc.y - state->next_y) < SAME_REGION_R);
}

bool rightAngle(state_t *state, double &angle) {
    if (state->next_x != state->loc.x) 
        angle = atan((state->next_y - state->loc.y)/(state->next_x - state->loc.x));
    else if (state->next_y > state->loc.y)
        angle = 3.14/2.0;
    else 
        angle = -3.14/2.0;
    
    return (abs(state->loc.theta - angle) < SAME_ANGLE_D);
}*/

// This thread continuously publishes command messages to the maebot
static void* send_cmds(void *data)
{
    state_t *state = (state_t *) data;
    uint32_t Hz = 20;
    //srand(time(NULL));

    //double sleep = rand()/RAND_MAX;
    //int forward = rand() % 4 + 1;
    //int count = 0;
    //int64_t utime = utime_now();

    while (state->running) {
        pthread_mutex_lock(&state->cmd_mutex);
        pthread_mutex_lock(&state->slam->path_mutex_);
/*        if(count%2)
        {
            rotateTowards(1, state);
        }
        else
        {
            moveForward(state);
        }
        int64_t nextTime = utime_now();
        sleep -= (nextTime - utime)/1000000.0;
        utime = nextTime;
        if(sleep <= 0)
        {
            sleep = rand()/RAND_MAX;
            count++;
        }
        if(forward <= 0)
        {
            forward = rand() % 6 + 3;
            count++;
        }

        pthread_mutex_unlock(&state->slam->path_mutex_);
        pthread_mutex_unlock(&state->cmd_mutex);
        usleep(1000000/Hz);*/
        /*if(!state->slam->bfs_result.empty())
        {
            int nextCell = state->slam->bfs_result.back();
            pthread_mutex_unlock(&state->slam->path_mutex_);

            moveTowardsPoint(state->slam->mostProbableParticle(), nextCell, state);
            pthread_mutex_unlock(&state->cmd_mutex);

            usleep(1000000/Hz);
        }
        else
        {
        }*/
        /*iusleep(1000000/Hz);
        
        pthread_mutex_lock(&state->loc.move_mutex);
        
        if (!state->has_feedback) {
            pthread_mutex_unlock(&state->loc.move_mutex);
            continue;
        }
        
        if (state->need_init){
            state->right_prev_dist = (state->feedback.encoder_right_ticks/480.0) * 0.032 * 3.14;
            state->left_prev_dist = (state->feedback.encoder_left_ticks/480.0) * 0.032 * 3.14;
            state->need_init = false;
        }
        
        double cur_dist = (state->feedback.encoder_right_ticks/480.0) * 0.032 * 3.14;
        double right_step = cur_dist - state->right_prev_dist;
        //printf("cur_dist: prev_dist: right_step:\t%f\t%f\t%f\n", cur_dist, state->right_prev_dist, right_step);
        state->right_prev_dist = cur_dist;
    
        cur_dist = (state->feedback.encoder_left_ticks/480.0) * 0.032 *3.14;
        double left_step = cur_dist - state->left_prev_dist;
        //printf("cur_dist: prev_dist: left_step:\t%f\t%f\t%f\n", cur_dist, state->left_prev_dist, left_step);
        state->left_prev_dist = cur_dist;

        //odometry
        double s_ = (right_step + left_step)/2;
        double delta_theta = (right_step - left_step)/0.08;
        double alpha = delta_theta/2;
        state->loc.x += cos(state->loc.theta + alpha) * s_; 
        state->loc.y += sin(state->loc.theta + alpha) * s_; 
        state->loc.theta = eecs467::wrap_to_2pi(state->loc.theta + delta_theta);

        maebot_motor_command_t cmd;
        cmd.utime = utime_now();
        if(reachGoal(state)){
            if (state->slam->bfs_result.empty()) {   
                pthread_mutex_unlock(&state->loc.move_mutex);
                cmd.motor_right_speed = 0.0f;
                cmd.motor_left_speed = 0.0f;
                state->lcm->publish("MAEBOT_MOTOR_COMMAND", &cmd);
                continue;
            }

            // get next goal location here
            state->next_x = state->grid_mapper->toX(state->slam->bfs_result.back());
            state->next_y = state->grid_mapper->toY(state->slam->bfs_result.back());
            state->slam->bfs_result.pop_back();
        }
        pthread_mutex_unlock(&state->loc.move_mutex);

        double angle;
        if(!rightAngle(state, angle)){
            if(eecs467::angle_diff(state->loc.theta, angle) > 0) {
                cmd.motor_right_speed = 0.8*FORWARD_V;
                cmd.motor_left_speed = 0.8*FORWARD_V * -1;
            } else {
                cmd.motor_right_speed = 0.8*FORWARD_V * -1;
                cmd.motor_left_speed = 0.8*FORWARD_V; 
            }
        } else { 
            cmd.motor_right_speed = FORWARD_V;
            cmd.motor_left_speed = FORWARD_V;
        }

        //state->lcm->publish("MAEBOT_MOTOR_COMMAND", &(cmd));*/
    }

    return NULL;
}

static void* update_map(void *data)
{
    state_t *state = (state_t*) data;

    state->slam->lockSlamMutex();
    while(!state->slam->scanReceived())
    {
        state->slam->wait();
    }
    state->slam->unlockSlamMutex();
    state->slam->pushFirstScan();
    LaserScan updated_scan = state->grid_mapper->calculateLaserOrigins();
    if(!updated_scan.valid)
    {
        std::cout << "initial scan not working" << std::endl;
        exit(1);
    }

    state->grid_mapper->updateGrid(updated_scan);
    state->grid_mapper->publishOccupancyGrid(updated_scan.end_pose);
    std::cout << "here" << std::endl;
    while(state->running)
    {
        state->slam->lockSlamMutex();
        while(!state->slam->scanReceived())
        {
            state->slam->wait();
        }
        state->slam->unlockSlamMutex();
        maebot_laser_scan_t next_scan = state->slam->updateParticles();
        //std::cout << "received scan" << std::endl;
        
        state->grid_mapper->addLaserScan(next_scan);
        //std::cout << "added laser scan" << std::endl;
        
        LaserScan updated_scan = state->grid_mapper->calculateLaserOrigins();
        //std::cout << "updated scan" << std::endl;
        if(!updated_scan.valid) exit(1);

        pthread_mutex_lock(&state->slam->path_mutex_);
        std::vector<int> retval = state->grid_mapper->updateGrid(updated_scan);
        state->slam->bfs_result.clear();
        state->slam->bfs_result = retval;
        pthread_mutex_unlock(&state->slam->path_mutex_);
        //std::cout << "update grid" << std::endl;
        state->grid_mapper->publishOccupancyGrid(updated_scan.end_pose);
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
    pthread_mutex_init(&state->loc.move_mutex, NULL);

    state->loc.x = 0;
    state->loc.y = 0;
    state->loc.theta = 0;
    state->next_x = 0;
    state->next_y = 0;
    state->need_init = true;
    state->has_feedback = false; 

    //feenableexcept(FE_DIVBYZERO| FE_INVALID|FE_OVERFLOW); 
    feenableexcept(FE_ALL_EXCEPT & ~FE_INEXACT & ~FE_UNDERFLOW);

    state->grid_mapper = new OccupancyGridMapper(state->lcm);
    state->slam = new Slam(state->grid_mapper, state->lcm, &state->loc);
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
    MaebotLCMHandler lcm_handler(state->grid_mapper, state->slam, state);

    state->lcm->subscribe("MAEBOT_LASER_SCAN",
            &MaebotLCMHandler::handleLaserScan,
            &lcm_handler);

    state->lcm->subscribe("MAEBOT_MOTOR_FEEDBACK",
            &MaebotLCMHandler::handleMotorFeedback,
            &lcm_handler);

    /*state->lcm->subscribe("MAEBOT_POSE_BEST",
            &MaebotLCMHandler::handlePose,
            &lcm_handler);*/

    std::cout << "listening" << std::endl;

    // Spin up threads
    //pthread_create(&state->cmd_thread, NULL, send_cmds, (void*)state);
    pthread_create(&state->update_map_thread, NULL, update_map, state);

    // Loop forever
    while(state->lcm->handle() == 0);

    pthread_join (state->update_map_thread, NULL);
    pthread_join (state->cmd_thread, NULL);

    return 0;
}
