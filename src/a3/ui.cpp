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
class state_t{
    public:
        vx_state_t camera_vx;
        vx_state_t og_vx; 
        char       *camera_url;
        //image_processor im_processor;
        pthread_mutex_t data_mutex;
        pthread_t       camera_animate_thread;
    public:
        state_t(){
            pthread_mutex_init (&data_mutex,NULL);
        }
        ~state_t(){
            pthread_mutex_destroy(&data_mutex);
        }
};

static void* camera_vx_render_loop(void *data){
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
        vx_buffer_t *buf = vx_world_get_buffer(state->camera_vx.vxworld,"image");
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

    pthread_create(&state.camera_animate_thread,NULL,camera_vx_render_loop,&state);

    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

    state.camera_vx.appwrap = vx_gtk_display_source_create(&state.camera_vx.vxapp);
    GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas = vx_gtk_display_source_get_widget(state.camera_vx.appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window), 640, 480);
    gtk_container_add(GTK_CONTAINER(window), canvas);
    gtk_widget_show (window);
    gtk_widget_show (canvas); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    state.og_vx.appwrap = vx_gtk_display_source_create(&state.og_vx.vxapp);
    GtkWidget * window2 = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    GtkWidget * canvas2 = vx_gtk_display_source_get_widget(state.og_vx.appwrap);
    gtk_window_set_default_size (GTK_WINDOW (window2), 640, 480);
    gtk_container_add(GTK_CONTAINER(window2), canvas2);
    gtk_widget_show (window2);
    gtk_widget_show (canvas2); // XXX Show all causes errors!
    g_signal_connect_swapped(G_OBJECT(window2), "destroy", G_CALLBACK(gtk_main_quit), NULL);

    gtk_main(); // Blocks as long as GTK window is open

    gdk_threads_leave();
    vx_gtk_display_source_destroy(state.camera_vx.appwrap);
    vx_gtk_display_source_destroy(state.og_vx.appwrap);
    vx_global_destroy();
}
