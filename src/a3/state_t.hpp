#ifndef _STATE_T_HPP_
#define _STATE_T_HPP_

#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <vector>
#include <deque>
#include <iostream>
#include <string>
#include "math/point.hpp"
#include "inttypes.h"
// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "common/pg.h"
// drawables
#include "vx/vxo_drawables.h"

//common
#include "common/getopt.h"
#include "common/timestamp.h"
#include "common/zarray.h"

//imagesource
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"
//a3
#include "vx_state_t.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "maebot_gui_t.hpp"
#include "calibration.hpp"
#include "image_processor.hpp"
#include "Navigator.hpp"
#include "lcmtypes/ui_dest_list_t.hpp"

class state_t{
    public:
        lcm::LCM                lcm;
        pthread_t               lcm_thread_pid;
        vx_state_t              camera_vx;
        vx_state_t              occupancy_grid_vx;
        vx_application_t        vxapp;
        vx_world_t              *vxworld;
        parameter_gui_t         *pg;
        parameter_listener_t    *my_listener;
        zhash_t                 *layers;
        char                    *camera_url;

        maebot_color            maebot_curr_selected;
        std::vector<maebot_gui_t> maebot_list;
        eecs467::OccupancyGrid   occupancy_grid;
        image_processor         im_processor;
        calibration_t           calibration;
        std::vector<float>        im_mask;
        Navigator               nav;

        pthread_mutex_t         mutex;
        pthread_mutex_t         data_mutex;
        pthread_t               animate_thread;
    public:
        state_t():camera_vx("camera frame"),occupancy_grid_vx("Occupancy Grid frame"), nav("../ground_truth/vmap.txt"){
            vxworld = vx_world_create();
            vxapp.impl = this;
            vxapp.display_started = display_started;
            vxapp.display_finished = display_finished;
            layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
            pg = pg_create();
            pg_add_buttons(pg,"red maebot","select red maebot","deselect maebot","deselect maebot","confirm route","publish path","delete waypoint","delete latest path",NULL);
            my_listener = (parameter_listener_t*) calloc (1, sizeof(*my_listener));
            my_listener->impl = this;
            my_listener->param_changed = my_param_changed;
            pg_add_listener (pg, my_listener);
            /*TODO*/
            //read calibration files
            calibration.read_image_to_world_mat("../calibration/im_to_world.txt");
            calibration.read_og_to_world_mat("../calibration/og_to_world.txt");
            calibration.read_world_to_og_mat("../calibration/world_to_og.txt");
            calibration.read_world_to_image_mat("../calibration/world_to_im.txt");
            calibration.read_mask("../calibration/mask_rect.txt");
            im_mask = calibration.get_mask();
            maebot_curr_selected = NONE;
            maebot_list.resize(NUM_MAEBOT);

            maebot_gui_t m;
            m.color = RED;
            m.curr_pos = {DBL_MAX,DBL_MAX};
            m.curr_dest = {DBL_MAX,DBL_MAX};
            m.particle_pos = {DBL_MAX,DBL_MAX};
            m.hsv_range.read_hsv_from_file("../calibration/blue_maebot_hsv_range.txt");

            maebot_list[RED] = m;
            occupancy_grid = eecs467::OccupancyGrid(5.0,5.0,0.05);
            read_map("../ground_truth/a_map.txt");
            pthread_mutex_init(&mutex,NULL);
            pthread_mutex_init(&data_mutex,NULL);
            lcm.subscribe("MAEBOT_DEST",&state_t::maebot_dest_handler,this);
            lcm.subscribe("MAEBOT_LOCALIZATION_RED",&state_t::maebot_best_particle_handler,this);
        }

        ~state_t(){
            free(my_listener);
            pg_destroy(pg);
            pthread_mutex_destroy(&mutex);
            pthread_mutex_destroy(&data_mutex);
        }

        void maebot_best_particle_handler(const lcm::ReceiveBuffer* rbuf,const std::string& channel, const maebot_pose_t *msg){
            pthread_mutex_lock(&data_mutex);
            if(channel == "MAEBOT_LOCALIZATION_RED"){
               maebot_list[RED].particle_pos.x = msg->x;
               maebot_list[RED].particle_pos.y = msg->y; 
               std::cout<<"receive best particle"<<msg->x<<" "<<msg->y<<std::endl;
            }
            if(channel == "MAEBOT_LOCALIZATION_BLUE"){
               maebot_list[BLUE].particle_pos.x = msg->x;
               maebot_list[BLUE].particle_pos.y = msg->y; 
            }
            if(channel == "MAEBOT_LOCALIZATION_GREEN"){
               maebot_list[GREEN].particle_pos.x = msg->x;
               maebot_list[GREEN].particle_pos.y = msg->y; 
            }
            pthread_mutex_unlock(&data_mutex);
        }

        void maebot_dest_handler(const lcm::ReceiveBuffer* rbuf,const std::string& channel,const ui_dest_list_t *msg){
            if(msg->num_way_points != 1){
                printf("Maebot Dest receive more than one dest, skip message\n");
                return;
            } 
            if(msg->color == 3){
                printf("Maebot color out of bound, skip message\n");
                return;
            }
            pthread_mutex_lock(&data_mutex);
            maebot_list[msg->color].curr_dest.x = msg->x_poses.front();
            maebot_list[msg->color].curr_dest.y = msg->y_poses.front();
            printf("Receive dest for Maebot %d with dest (%f,%f)\n",msg->color,msg->x_poses.front(),msg->y_poses.front());
            pthread_mutex_unlock(&data_mutex);
        }

        static void my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
        {
            state_t *state = (state_t*) pl->impl;
            pthread_mutex_lock(&state->data_mutex);
            if (0==strcmp ("red maebot", name)){
                state->maebot_curr_selected = RED;
                printf ("red\n");
            }
            else if (0==strcmp ("deselect maebot",name)){
                state->maebot_curr_selected = NONE;
                printf("deselect maebot\n");
            }
            else if (0==strcmp ("confirm route", name)){
                if(state->maebot_curr_selected != NONE && !state->maebot_list[state->maebot_curr_selected].waypoints.empty()){
                    //state->maebot_list[state->maebot_curr_selected].curr_dest = state->maebot_list[state->maebot_curr_selected].waypoints.front();
                    //state->maebot_list[state->maebot_curr_selected].waypoints.pop_front();
                    // TODO: publish lcm message
                    ui_dest_list_t list_from_ui;
                    list_from_ui.color = state->maebot_curr_selected;
                    list_from_ui.num_way_points = state->maebot_list[state->maebot_curr_selected].waypoints.size();
                    for(int i=0;i<list_from_ui.num_way_points;++i){
                        list_from_ui.x_poses.push_back(state->maebot_list[state->maebot_curr_selected].waypoints[i].x);
                        list_from_ui.y_poses.push_back(state->maebot_list[state->maebot_curr_selected].waypoints[i].y);
                    }
                    state->maebot_list[state->maebot_curr_selected].waypoints.clear();
                    state->lcm.publish("UI_DEST_LIST",&list_from_ui);
                    printf ("publish\n");
                }
            }
            else if (0==strcmp ("delete waypoint", name)){
                if(state->maebot_curr_selected != NONE && !state->maebot_list[state->maebot_curr_selected].waypoints.empty()){
                    state->maebot_list[state->maebot_curr_selected].waypoints.pop_back();
                    printf ("delete\n");
                }
            }
            else{
                printf ("%s changed\n", name);
            }
            pthread_mutex_unlock(&state->data_mutex);
        }

        void read_map(char const *addr){
            FILE *fp;
            uint t;
            uint8_t temp;
            fp = fopen(addr,"r");
            if(fp == NULL){
                printf("File not exist\n");
                exit(1);
            }
            fscanf(fp,"%d\n",&t);
            if(t != occupancy_grid.heightInCells()){
                std::cout << "Height not match\n";
                exit(1);
            }
            fscanf(fp,"%d\n",&t);
            if(t != occupancy_grid.widthInCells()){
                std::cout << "Width not match\n";
                exit(1);
            }
            for(size_t y = 0; y < occupancy_grid.heightInCells();y++){
                for(size_t x = 0; x < occupancy_grid.widthInCells(); x++){
                    fscanf(fp,"%d ",&temp);
                    occupancy_grid.setLogOdds(x,y,temp);
                }
            } 

        }
        static void display_finished(vx_application_t * app, vx_display_t * disp)
        {
            state_t * state = (state_t*)app->impl;
            pthread_mutex_lock(&state->mutex);

            vx_layer_t * layer = NULL;

            // store a reference to the world and layer that we associate with each vx_display_t
            zhash_remove(state->layers, &disp, NULL, &layer);

            vx_layer_destroy(layer);

            pthread_mutex_unlock(&state->mutex);
        }

        static void display_started(vx_application_t * app, vx_display_t * disp)
        {
            state_t * state = (state_t*)app->impl;

            vx_layer_t * layer = vx_layer_create(state->vxworld);
            vx_layer_set_display(layer, disp);
            //important setting for the handler
            //if(state->vxeh != NULL){
            //    vx_layer_add_event_handler(layer,state->vxeh);
            //}
            pthread_mutex_lock(&state->mutex);
            // store a reference to the world and layer that we associate with each vx_display_t
            zhash_put(state->layers, &disp, &layer, NULL, NULL);
            pthread_mutex_unlock(&state->mutex);
        } 
};

#endif
