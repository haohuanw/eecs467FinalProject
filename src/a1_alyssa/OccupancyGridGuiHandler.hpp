#ifndef OCCUPANCY_GRID_GUI_HANDLER_HPP
#define OCCUPANCY_GRID_GUI_HANDLER_HPP

#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>

#include "maebot_handlers.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "lcmtypes/maebot_occupancy_grid_t.hpp"

#include "state.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"

class OccupancyGridGuiHandler
{
    public:
		eecs467::OccupancyGrid *grid;
		
        OccupancyGridGuiHandler(eecs467::OccupancyGrid* _grid){
        	grid = _grid;
        }
        
        ~OccupancyGridGuiHandler(){}

        void handleMessage(const lcm::ReceiveBuffer *rbuf,
                           const std::string& channel,
                           const maebot_occupancy_grid_t *msg)
        {   
            //std::cout << "gui received grid" << std::endl;
            assert(channel == "OCCUPANCY_GRID_GUI");
            
            grid->reset();
            grid->fromLCM(*msg);
        }
};

class LocationHandler
{
    public:
        gui_state *state;
        
        LocationHandler(gui_state *state_t) : state(state_t) {}
        ~LocationHandler(){}

        void handlePose(const lcm::ReceiveBuffer *rbuf,
                        const std::string& channel,
                        const maebot_pose_t *msg)
        {
            //std::cout << "gui received pose" << std::endl;
            assert(channel == "MAEBOT_POSE_BEST");
            pthread_mutex_lock(&state->gui_mutex);
            state->poses.push_back(*msg);
            pthread_mutex_unlock(&state->gui_mutex);
        }

        /*void handleTruePose(const lcm::ReceiveBuffer *rbuf,
                            const std::string& channel,
                            const maebot_pose_t *msg)
        {
            pthread_mutex_lock(&state->gui_mutex);
            state->truePoses.push_back(*msg);
            pthread_mutex_unlock(&state->gui_mutex);
        }*/
        
        void handleParticles(const lcm::ReceiveBuffer *rbuf,
                            const std::string& channel,
                            const maebot_pose_t *msg)
        {
            pthread_mutex_lock(&state->gui_mutex);
            state->particles.push_back(*msg);
            pthread_mutex_unlock(&state->gui_mutex);
        }
        
        void handlePath(const lcm::ReceiveBuffer *rbuf,
                        const std::string& channel,
                        const maebot_pose_t *msg)
        {
            pthread_mutex_lock(&state->gui_mutex);
            state->pathPoints.push_back(*msg);
            pthread_mutex_unlock(&state->gui_mutex);
        }
};

class EndpointsHandler
{
    public:
		gui_state *state;
		
        EndpointsHandler(gui_state* _state){
        	state = _state;
        }
        
        ~EndpointsHandler(){}

        void handleEndpoints(const lcm::ReceiveBuffer *rbuf,
                             const std::string& channel,
                             const maebot_pose_t *msg)
        {   
            //std::cout << "gui received grid" << std::endl;
            //assert(channel == "OCCUPANCY_GRID_GUI");
            //std::cout << "received endpoints" << std::endl;
            
            pthread_mutex_lock(&state->gui_mutex);
            state->endpoints.push_back(*msg);
            pthread_mutex_unlock(&state->gui_mutex);
        }
};

#endif
