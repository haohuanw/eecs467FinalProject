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

//A2
#include "image_processor.hpp"

class state_t
{
    public:
        // vx stuff
        bool                running;
        getopt_t            *gopt;
        bool                usePic;
        char                *pic_url;
        char                *camera_url;
        image_processor     im_processor;
        vx_application_t    vxapp;
        vx_world_t          *vxworld;
        zhash_t             *layers;
        vx_event_handler_t  *vxeh;
        eecs467::Point<float> last_two_mouse_event;
        //read info from file
        max_min_hsv         red_hsv;
        max_min_hsv         green_hsv;
        max_min_hsv         cyan_hsv;
        eecs467::Point<float> corner_coords[2];
        int                 click_count;
        vx_mouse_event_t    last_mouse_event;
        vx_gtk_display_source_t* appwrap;
        pthread_mutex_t     data_mutex; // shared data lock
        pthread_mutex_t     mutex;
        pthread_t animate_thread;

    public:
        state_t()
        {
            //GUI init stuff
            vxworld = vx_world_create();
            printf("create world\n");
            vxeh = (vx_event_handler_t*) calloc(1,sizeof(*vxeh));
            vxeh->mouse_event = mouse_event;
            vxeh->touch_event = touch_event;
            vxeh->key_event = key_event;
            vxeh->dispatch_order = 100;
            vxeh->impl = this;
            printf("create vxeh\n");
            vxapp.impl= this;
            vxapp.display_started = display_started;
            vxapp.display_finished = display_finished;
            layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
            click_count = 0;
            pthread_mutex_init (&mutex, NULL);
            pthread_mutex_init (&data_mutex,NULL);
            running = 1;
            usePic = false;
            gopt = getopt_create(); 
            FILE *fp = fopen("../calibration/mask_rect.txt","r");
            fscanf(fp,"%f %f %f %f\n",&corner_coords[0].x,&corner_coords[1].x,&corner_coords[0].y,&corner_coords[1].y);
            printf("coord: %f %f %f %f\n",corner_coords[0].x,corner_coords[1].x,corner_coords[0].y,corner_coords[1].y);
            red_hsv.read_hsv_from_file("../calibration/red_maebot_range.txt");
            green_hsv.read_hsv_from_file("../calibration/green_hsv_range.txt");
	    cyan_hsv.read_hsv_from_file("../calibration/cyan_hsv_range.txt");
        }

        ~state_t()
        {
            vx_world_destroy(vxworld);
            if(zhash_size(layers) != 0){
                zhash_destroy(layers);
            }
            if(gopt){
                getopt_destroy(gopt);
            }
            pthread_mutex_destroy(&mutex);
        }

        void init_thread()
        {
            pthread_create(&animate_thread,NULL,&state_t::render_loop,this);
        }

        static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
        {
            state_t *state = (state_t*)vxeh->impl;
            pthread_mutex_lock(&state->data_mutex);
            // vx_camera_pos_t contains camera location, field of view, etc
            // vx_mouse_event_t contains scroll, x/y, and button click events
            if ((mouse->button_mask & VX_BUTTON1_MASK) &&
                    !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {
                vx_ray3_t ray;
                vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);
                double ground[3];
                vx_ray3_intersect_xy (&ray, 0, ground);
                printf ("Mouse clicked at coords: [%8.3f, %8.3f] Ground clicked at coords: [%6.3f, %6.3f]\n",
                        mouse->x, mouse->y, ground[0], ground[1]);
                if(state->click_count == 0){
                    state->last_two_mouse_event.x = ground[0];
                    state->last_two_mouse_event.y = ground[1];
                    state->click_count += 1;
                }
                else if(state->click_count == 1){
                    state->corner_coords[0].x = state->last_two_mouse_event.x+320;
                    state->corner_coords[0].y = state->last_two_mouse_event.y+240;
                    state->corner_coords[1].x = ground[0]+320;
                    state->corner_coords[1].y = ground[1]+240;
                    state->click_count += 1;
                }
                else{
                    state->click_count = 0;
                }
            }
            state->last_mouse_event = *mouse;
            pthread_mutex_unlock(&state->data_mutex);
            return 0;
        }
        static int key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
        {
            //state_t *state = vxeh->impl;
            return 0;
        }

        static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
        {
            return 0; // Does nothing
        }
        static void* render_loop(void* data)
        {
            state_t * state = (state_t*) data;
            int fps = 60;
            image_source_t *isrc = NULL;
            if(!state->usePic){
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
            }
            while (state->running) {
                pthread_mutex_lock(&state->data_mutex);
                vx_buffer_t *buf = vx_world_get_buffer(state->vxworld,"image");
                image_u32_t *im; 
                std::vector<int> red_center_list;
		std::vector<int> green_center_list;
		std::vector<int> cyan_center_list;
                if(state->usePic){
                    im = image_u32_create_from_pnm(state->pic_url); 
                }
                else if(isrc != NULL){
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
                if(state->corner_coords[0].x != -1 && state->corner_coords[0].y != -1 && state->corner_coords[1].x != -1 && state->corner_coords[1].y != -1){  
		            state->im_processor.image_masking(im,state->corner_coords[0].x,state->corner_coords[1].x,state->corner_coords[0].y,state->corner_coords[1].y);
                    red_center_list = state->im_processor.blob_detection(im,state->corner_coords[0].x,state->corner_coords[1].x,state->corner_coords[0].y,state->corner_coords[1].y,state->red_hsv);
		            green_center_list = state->im_processor.blob_detection(im,state->corner_coords[0].x,state->corner_coords[1].x,state->corner_coords[0].y,state->corner_coords[1].y,state->green_hsv); 
			    cyan_center_list = state->im_processor.blob_detection(im, state->corner_coords[0].x,state->corner_coords[1].x,state->corner_coords[0].y,state->corner_coords[1].y,state->cyan_hsv); 
			    //printf("numCenters %d %d %d\n",red_center_list.size(),green_center_list.size(), cyan_center_list.size());
		} 
		if(!red_center_list.empty()){
		    for(int i=0;i<red_center_list.size();++i){
			    int y = (red_center_list[i])/im->width;
			    int x = (red_center_list[i])%im->width;
			    state->im_processor.draw_circle(im,x,y,10.0,0xff0000ff);
		    }
		}
		if(!green_center_list.empty()){
		    for(int i=0;i<green_center_list.size();++i){
			int y = (green_center_list[i])/im->width;
			int x = green_center_list[i]%im->width;
			state->im_processor.draw_circle(im,x,y,10.0,0xff00ff00);
		    }
		}
		if(!cyan_center_list.empty()){
		  for(int i=0; i < cyan_center_list.size(); ++i){
		    int y = (cyan_center_list[i]) / im->width;
		    int x = cyan_center_list[i] % im->width;
		    state->im_processor.draw_circle(im, x, y, 20.0, 0xffffff00);
		  }
		}
                if(im != NULL){
                    vx_object_t *vim = vxo_image_from_u32(im,
                            VXO_IMAGE_FLIPY,
                            VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
                    //use pix coords to make a fix image
                    vx_buffer_add_back (buf,vxo_chain (
                                    vxo_mat_translate3 (-im->width/2., -im->height/2., 0.),
                                    vim));
                    image_u32_destroy (im);
                }
                if(state->click_count == 2){
                    state->click_count = 0;
                }
                pthread_mutex_unlock(&state->data_mutex);
                vx_buffer_swap(buf);
                usleep(1000000/fps);
            }
            return NULL;
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

int main(int argc, char ** argv)
{
    state_t state;
    printf("init state\n");
    //find camera urls and use the first one
    getopt_add_string (state.gopt, 'f', "picurl", "", "Picture URL");

    if (!getopt_parse (state.gopt, argc, argv, 1)) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state.gopt);
        exit (EXIT_FAILURE);
    }

    if (strncmp (getopt_get_string (state.gopt, "picurl"), "", 1)) {
        state.pic_url = strdup (getopt_get_string (state.gopt, "picurl"));
        state.usePic = true;
        printf ("URL: %s\n", state.pic_url);
    }
    else{
        printf("camera find\n");
        zarray_t *urls = image_source_enumerate();
        if(0 == zarray_size(urls)){
            printf("No camera found.\n");
            exit(1);
        }
        printf("find camera\n");
        zarray_get(urls,0,&state.camera_url);
        printf("get camera address\n");
    }
    state.init_thread();
    //vx
    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    state.appwrap = vx_gtk_display_source_create(&state.vxapp);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(state.appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 640, 480);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);
    gtk_main(); // Blocks as long as GTK window is open

    gdk_threads_leave();
    vx_gtk_display_source_destroy(state.appwrap);
    //vx
    state.running = 0;
    pthread_join(state.animate_thread,NULL);

    vx_global_destroy();
}
