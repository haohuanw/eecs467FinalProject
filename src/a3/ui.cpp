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
// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"

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

class state_t{
    public:
        //vx_state_t camera_vx;
        //vx_state_t og_vx;
        vx_state_t              camera_vx;
        vx_state_t              occupancy_grid_vx;
        /*TODO
         * add vxapp vxworld layers and pg listener to initilize third
         * window
         * add maebot list and calibration
         * maebot should be a struct with point, waypoints(in world coord), path and hsv range
         * add maebot_button_selected to be able to switch between
         * different maebot
         */
        char                    *camera_url;
        eecs467::OccupancyGrid   occupancy_grid;
        //image_processor im_processor;
        pthread_mutex_t         mutex;
        pthread_mutex_t         data_mutex;
        pthread_t               animate_thread;
    public:
        state_t():camera_vx("camera frame"),occupancy_grid_vx("Occupancy Grid frame"){
            //GUI init
            //camera_vx.vx_world = vx_world_create();
            //occupancy_grid_vx.vx_world = vx_world_create();
            /*occupancy_grid_eh = (vx_event_handler_t*) calloc(1,sizeof(*occupancy_grid_eh));*/
            //camera_vx.vx_app.impl = this;
            //camera_vx.vx_app.display_started = display_started;
            //camera_vx.vx_app.display_finished = display_finished;

            //occupancy_grid_vx.vx_app.impl = this;
            //occupancy_grid_vx.vx_app.display_started = display_started;
            //occupancy_grid_vx.vx_app.display_finished = display_finished;

            //layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
            /*TODO
            * initGUI
            *
            * init maebot list
            * init pg
            */
            occupancy_grid = eecs467::OccupancyGrid(5.0,5.0,0.05);
            read_map("../ground_truth/figure_eight.txt");
            pthread_mutex_init(&mutex,NULL);
            pthread_mutex_init(&data_mutex,NULL);
        }

        ~state_t(){
            //vx_world_destroy(camera_vx.vx_world);
            //vx_world_destroy(occupancy_grid_vx.vx_world);
            /*TODO
             *Destory accordingly
             *
             *
             */       
            pthread_mutex_destroy(&mutex);
            pthread_mutex_destroy(&data_mutex);
        }
        /*TODO
        * add pg handler here
        */
        void read_map(char *addr){
            FILE *fp;
            uint8_t temp;
            fp = fopen(addr,"r");
            if(fp == NULL){
                printf("File not exist\n");
                exit(1);
            }
            fscanf(fp,"%d\n",&temp);
            if(temp != occupancy_grid.heightInCells()){
                std::cout << "Height not match\n";
                exit(1);
            }
            fscanf(fp,"%d\n",&temp);
            if(temp != occupancy_grid.widthInCells()){
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

};

static void draw(state_t *state){
    vx_buffer_t *buf = vx_world_get_buffer(state->occupancy_grid_vx.vxworld,"og_map");
    image_u8_t *og_im = image_u8_create(state->occupancy_grid.widthInCells(),state->occupancy_grid.heightInCells());
    for (size_t y = 0; y < state->occupancy_grid.heightInCells(); y++)
    {
        for (size_t x = 0; x < state->occupancy_grid.widthInCells(); x++)
        {
            og_im->buf[(y*og_im->stride) + x] = 127 - state->occupancy_grid.logOdds(x,y);
        }
    } 
    eecs467::Point<float> origin = state->occupancy_grid.originInGlobalFrame();
    vx_object_t *vo = vxo_chain(vxo_mat_translate3(-og_im->width/2.0,-og_im->height/2.0,0.0),
            vxo_image_from_u8(og_im,0,0));
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
        vx_buffer_t *buf = vx_world_get_buffer(state->camera_vx.vxworld,"camera");
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
        eecs467::Point<float> camera_click_point = state->camera_vx.get_click_point();
        if(camera_click_point.x != -1 && camera_click_point.y!= -1){
            /*TODO
            * add way point accordingly to the vector with the transformation
            */
            printf("Camera frame: added a way point at: %6.3f %6.3f\n",camera_click_point.x,camera_click_point.y);
        }
        eecs467::Point<float> og_click_point = state->occupancy_grid_vx.get_click_point();
        if(og_click_point.x != -1 && og_click_point.y!= -1){
            /*TODO
            * add way point accordingly to the vector with the transformation
            */
            printf("Occupancy Grid frame: added a way point at: %6.3f %6.3f\n",og_click_point.x,og_click_point.y);
        }
        /*TODO
        * if there is a way point of selected maebot, display it
        */
        /*TODO
        * has way points? path planning to calculate the path
        * cat them in one path
        * and display it
        */
        if(im != NULL){
            vx_object_t *vim = vxo_image_from_u32(im,
                    VXO_IMAGE_FLIPY,
                    VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
            //use pix coords to make a fix image
            vx_buffer_add_back (buf,
                    vxo_chain (
                        vxo_mat_translate3 (-im->width/2., -im->height/2., 0.),
                        vim));
            image_u32_destroy (im);
        }
        pthread_mutex_unlock(&state->data_mutex);
        vx_buffer_swap(buf);
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

    gtk_main(); // Blocks as long as GTK window is open

    gdk_threads_leave();
    vx_gtk_display_source_destroy(appwrap);
    vx_global_destroy();
}
