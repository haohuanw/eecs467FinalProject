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
#define NUM_MAEBOT 1

class state_t{
    public:
        vx_state_t              camera_vx;
        vx_state_t              occupancy_grid_vx;
        vx_application_t        vxapp;
        vx_world_t              *vxworld;
        parameter_gui_t         *pg;
        parameter_listener_t    *my_listener;
        zhash_t                 *layers;
        /*TODO
         * add maebot list and calibration
         * maebot should be a struct with point, waypoints(in world coord), path and hsv range
         * add maebot_button_selected to be able to switch between
         * different maebot
         */
        maebot_color            maebot_curr_selected;
        std::vector<maebot_gui_t> maebot_list;
        char                    *camera_url;
        eecs467::OccupancyGrid   occupancy_grid;
        image_processor         im_processor;
        calibration_t           calibration;
        std::vector<int>        im_mask;
        pthread_mutex_t         mutex;
        pthread_mutex_t         data_mutex;
        pthread_t               animate_thread;
    public:
        state_t():camera_vx("camera frame"),occupancy_grid_vx("Occupancy Grid frame"){
            vxworld = vx_world_create();
            vxapp.impl = this;
            vxapp.display_started = display_started;
            vxapp.display_finished = display_finished;
            layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
            pg = pg_create();
            pg_add_buttons(pg,"red","red maebot","publish","publish path","delete","delete latest path",NULL);
            my_listener = (parameter_listener_t*) calloc (1, sizeof(*my_listener));
            my_listener->impl = this;
            my_listener->param_changed = my_param_changed;
            pg_add_listener (pg, my_listener);
            //read calibration files
            /*calibration.read_image_to_world_mat("");
              calibration.read_og_to_world_mat("");
              calibration.read_world_to_og_mat("");
              calibration.read_world_to_image_mat("");
              calibration.read_mask("")
              im_mask = calibration.get_mask()*/
            maebot_curr_selected = NONE;
            maebot_list.resize(NUM_MAEBOT);

            maebot_gui_t m;
            m.color = RED;
            m.curr_pos = {DBL_MAX,DBL_MAX};
            m.hsv_range.read_hsv_from_file("../calibration/red_maebot_range.txt");

            maebot_list[RED] = m;
            occupancy_grid = eecs467::OccupancyGrid(5.0,5.0,0.05);
            read_map("../ground_truth/figure_eight.txt");
            pthread_mutex_init(&mutex,NULL);
            pthread_mutex_init(&data_mutex,NULL);
        }

        ~state_t(){
            free(my_listener);
            pg_destroy(pg);
            pthread_mutex_destroy(&mutex);
            pthread_mutex_destroy(&data_mutex);
        }
        static void my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
        {
            state_t *state = (state_t*) pl->impl;
            pthread_mutex_lock(&state->data_mutex);
            if (0==strcmp ("red", name)){
                state->maebot_curr_selected = RED;
                printf ("red\n");
            }
            else if (0==strcmp ("publish", name)){
                if(state->maebot_curr_selected != NONE && !state->maebot_list[state->maebot_curr_selected].waypoints.empty()){
                    state->maebot_list[state->maebot_curr_selected].waypoints.pop_front();
                    printf ("publish\n");
                }
            }
            else if (0==strcmp ("delete", name)){
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

static void draw(state_t *state){
    vx_buffer_t *buf = vx_world_get_buffer(state->occupancy_grid_vx.vxworld,"og_map");
    image_u8_t  *og_im = image_u8_create(state->occupancy_grid.widthInCells(),state->occupancy_grid.heightInCells());
    for (size_t y = 0; y < state->occupancy_grid.heightInCells(); y++)
    {
        for (size_t x = 0; x < state->occupancy_grid.widthInCells(); x++)
        {
            og_im->buf[(y*og_im->stride) + x] = 127 - state->occupancy_grid.logOdds(x,y);
        }
    } 
    //eecs467::Point<float> origin = state->occupancy_grid.originInGlobalFrame();
    vx_object_t *vo = vxo_chain(vxo_mat_translate3(-og_im->width/2.0,-og_im->height/2.0,0.0),vxo_image_from_u8(og_im,0,0));
    vx_buffer_add_back(buf,vo);
    image_u8_destroy(og_im);
    vx_buffer_swap(buf); 
}

static void* render_loop(void *data){
    state_t *state = (state_t*)data;
    int fps = 60;
    image_source_t *isrc = NULL;
    isrc = image_source_open(state->camera_url);
    if(isrc == NULL){
        printf("Error opening device\n");
    }
    else{
        /*for (int i = 0; i < isrc->num_formats (isrc); i++) {
          image_source_format_t ifmt;
          isrc->get_format (isrc, i, &ifmt);
          printf ("%3d: %4d x %4d (%s)\n",
          i, ifmt.width, ifmt.height, ifmt.format);
          }*/
        isrc->start(isrc);
        //printf("isrc->start");
    }
    while(1){
        pthread_mutex_lock(&state->data_mutex);
        vx_buffer_t *cam_buf = vx_world_get_buffer(state->camera_vx.vxworld,"camera");
        vx_buffer_t *control_buf = vx_world_get_buffer(state->vxworld,"control");
        image_u32_t *im; 
        if(isrc != NULL){
            image_source_data_t *frmd = (image_source_data_t*) calloc(1,sizeof(*frmd));
            int res = isrc->get_frame(isrc,frmd);
            if(res < 0){
                printf("get_frame fail\n");
            }
            else{
                im = image_convert_u32(frmd);
            }
            fflush(stdout);
            isrc->release_frame(isrc,frmd);
        }
        /*TODO
         *add mask and blob detection
         */
        /*if(state->im_mask[0] != -1 && state->im_mask[1] != -1 && state->im_mask[2] != -1 && state->im_mask[3] != -1){
            state->im_processor.image_masking(im,state->im_mask[0],state->im_mask[1],state->im_mask[2],state->im_mask[3]);
            for(int i=0;i<NUM_MAEBOT;++i){ 
                int pos = state->im_processor.blob_detection(im,state->im_mask[0],state->im_mask[1],state->im_mask[2],state->im_mask[3],state->maebot_list[i].hsv_range);
                state->maebot_list[i].curr_pos.x = cali.trans(pos % im->width);
                state->maebot_list[i].curr_pos.y = cali.trans(pos / im->width); 
            }
            for(int i=0;i<NUM_MAEBOT;++i){
                trans curr_pos to im coord
                trans curr_pos to og coord 
                state->im_processor.draw_circle(im,state->maebot_list[i].curr_pos.x,state->maebot_list[i].curr_pos.y,10.0,state->maebot_list[i].disp_color);
                int npoints = 1;
                float points[3] = {og_coord}
                vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                vx_buffer_add_back(og_buf, vxo_points(verts, npoints, vxo_points_style(vx_green, 2.0f)));
            }
        }*/  
        eecs467::Point<float> camera_click_point = state->camera_vx.get_click_point();
        if(camera_click_point.x != -1 && camera_click_point.y!= -1){
            /*TODO
             * add way point accordingly to the vector with the transformation
             */
            if(state->maebot_curr_selected != NONE){
                //state->maebot_list[state->maebot_curr_selected].waypoints.push_back(/*point after trans*/);
                //path planning
                printf("Camera frame: added a way point at: %6.3f %6.3f\n",camera_click_point.x,camera_click_point.y);
                state->maebot_list[state->maebot_curr_selected].waypoints.push_back(eecs467::Point<double>{(double)camera_click_point.x,(double)camera_click_point.y});
            }
        }
        eecs467::Point<float> og_click_point = state->occupancy_grid_vx.get_click_point();
        if(og_click_point.x != -1 && og_click_point.y!= -1){
            /*TODO
             * add way point accordingly to the vector with the transformation
             */
            if(state->maebot_curr_selected != NONE){
                //state->maebot_list[state->maebot_curr_selected].waypoints.push_back(/*point after trans*/);
                //path planing
                printf("Occupancy Grid frame: added a way point at: %6.3f %6.3f\n",og_click_point.x,og_click_point.y);
                state->maebot_list[state->maebot_curr_selected].waypoints.push_back(eecs467::Point<double>{(double)og_click_point.x,(double)og_click_point.y});
            }
        }

        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,"<<center,#000000,serif>>Current Maebot");
        vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-580,30),vxo_mat_scale(0.8),vt)));

        if(state->maebot_curr_selected != NONE){
            //display in the control plane
            if(state->maebot_curr_selected == RED){
                vx_object_t *vm = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,"<<center,#ff0000,serif>>RED");
                vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-580,-30),vxo_mat_scale(0.8),vm)));
            } 
            for(uint i=0;i<state->maebot_list[state->maebot_curr_selected].waypoints.size();++i){
                char x[31];
                char y[31];
                snprintf(x,31,"<<center,#000000,serif>>%f",state->maebot_list[state->maebot_curr_selected].waypoints[i].x);
                snprintf(y,31,"<<center,#000000,serif>>%f",state->maebot_list[state->maebot_curr_selected].waypoints[i].y); 
                vx_object_t *vwx = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,x);
                vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-490+(int)i*60,30),vxo_mat_scale(0.8),vwx)));
                vx_object_t *vwy = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,y);
                vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-490+(int)i*60,-30),vxo_mat_scale(0.8),vwy)));
            }
            //display the way points
            if(!state->maebot_list[state->maebot_curr_selected].waypoints.empty()){
                int npoints = state->maebot_list[state->maebot_curr_selected].waypoints.size();
                float points[npoints*3];
                for(uint i=0;i<state->maebot_list[state->maebot_curr_selected].waypoints.size();++i){
                    points[i*3+0] = state->maebot_list[state->maebot_curr_selected].waypoints[i].x;
                    points[i*3+1] = state->maebot_list[state->maebot_curr_selected].waypoints[i].y;
                    points[i*3+2] = 2;
                    //points[i*3+0] = world_to_im;
                    //points[i*3+1] = world_to_im;
                    //points[i*3+2] = 0;
                }
                vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                vx_buffer_add_back(cam_buf, vxo_points(verts, npoints, vxo_points_style(vx_green, 4.0f)));
            }
            /*TODO*/
            //display the path
            //same for og_buf
        }
        if(im != NULL){
            vx_object_t *vim = vxo_image_from_u32(im,
                    VXO_IMAGE_FLIPY,
                    VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
            //use pix coords to make a fix image
            vx_buffer_add_back (cam_buf,
                    vxo_chain (
                        vxo_mat_translate3 (-im->width/2., -im->height/2., 0.),
                        vim));
            image_u32_destroy (im);
        }
        pthread_mutex_unlock(&state->data_mutex);
        vx_buffer_swap(cam_buf);
        vx_buffer_swap(control_buf);
        usleep(1000000/fps);
    }
    return NULL;
}

int main(int argc, char ** argv){
    state_t state;
    printf("camera finding\n");
    zarray_t *urls = image_source_enumerate();
    if(0 == zarray_size(urls)){
        printf("No camera found.\n");
        exit(1);
    }
    printf("find camera\n");
    zarray_get(urls,0,&state.camera_url);
    printf("get camera address\n");
    draw(&state);
    pthread_create(&state.animate_thread,NULL,render_loop,&state);

    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    vx_gtk_display_source_t *appwrap = vx_gtk_display_source_create(&state.camera_vx.vxapp);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 640, 480);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    appwrap = vx_gtk_display_source_create(&state.occupancy_grid_vx.vxapp);
    GtkWidget * window2 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas2 = vx_gtk_display_source_get_widget(appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window2), 640, 480);
    gtk_container_add(GTK_CONTAINER(window2), canvas2);
    gtk_widget_show (window2);
    gtk_widget_show (canvas2); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window2), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    appwrap = vx_gtk_display_source_create(&state.vxapp);
    GtkWidget * window3 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas3 = vx_gtk_display_source_get_widget(appwrap);
    GtkWidget *pgui = pg_get_widget (state.pg);
    gtk_window_set_default_size (GTK_WINDOW (window3), 1280, 120);
    GtkWidget *vbox = gtk_vbox_new (0, 0);
    gtk_box_pack_start (GTK_BOX (vbox), canvas3, 1, 1, 0);
    gtk_box_pack_start (GTK_BOX (vbox), pgui, 0, 0, 0);
    gtk_container_add (GTK_CONTAINER (window3), vbox);
    gtk_widget_show (window3);
    gtk_widget_show (canvas3); // XXX Show all causes errors!
    gtk_widget_show (pgui);
    gtk_widget_show (vbox);
    g_signal_connect_swapped(G_OBJECT(window3), "destroy", G_CALLBACK(gtk_main_quit), NULL);


    gtk_main(); // Blocks as long as GTK window is open

    gdk_threads_leave();
    vx_gtk_display_source_destroy(appwrap);
    vx_global_destroy();
}
