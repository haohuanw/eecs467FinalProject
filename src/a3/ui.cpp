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
//lcmtypes
#include "lcmtypes/maebot_pose_t.hpp"
//a3
#include "vx_state_t.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "maebot_gui_t.hpp"
#include "calibration.hpp"
#include "image_processor.hpp"
#include "state_t.hpp"

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
        vx_buffer_t *og_buf = vx_world_get_buffer(state->occupancy_grid_vx.vxworld,"occupancy_grid");
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

        // do blob detection for all the maebots
        if(state->im_mask[0] != -1 && state->im_mask[1] != -1 && state->im_mask[2] != -1 && state->im_mask[3] != -1){
            // mask image
            state->im_processor.image_masking(im,state->im_mask[0],state->im_mask[1],state->im_mask[2],state->im_mask[3]);

            // detects each maebot
            for(int i=0;i<NUM_MAEBOT;++i){ 
                std::vector<int> pos;
                if(state->maebot_list[i].curr_pos.x != DBL_MAX && state->maebot_list[i].curr_pos.y != DBL_MAX)
                {
                    //curr_pos is based on the window coordinates
                    //blob detection returns the position based on the image file coordinates
                    eecs467::Point<double> curr_pos_im_coord = state->calibration.world_to_image_translate(state->maebot_list[i].curr_pos);
                    curr_pos_im_coord.x = im->width/2+curr_pos_im_coord.x;
                    curr_pos_im_coord.y = im->height - (im->height/2+curr_pos_im_coord.y);
                    pos = state->im_processor.blob_detection(im, curr_pos_im_coord.x - 200.0,
                            curr_pos_im_coord.x + 200.0,
                            curr_pos_im_coord.y - 200.0,
                            curr_pos_im_coord.y + 200.0,
                            state->maebot_list[i].hsv_range);
                    //std::cout<<"adpative way"<<std::endl;
                }
                if((state->maebot_list[i].curr_pos.x == DBL_MAX && state->maebot_list[i].curr_pos.y == DBL_MAX) || pos.empty()){
                    pos = state->im_processor.blob_detection(im,state->im_mask[0],state->im_mask[1],state->im_mask[2],state->im_mask[3],state->maebot_list[i].hsv_range);
                }
                else{
                    pos = state->im_processor.blob_detection(im,state->im_mask[0],state->im_mask[1],state->im_mask[2],state->im_mask[3],state->maebot_list[i].hsv_range);
                }
                if(!pos.empty()){
                    state->maebot_list[i].curr_pos = state->calibration.image_to_world_translate(eecs467::Point<double>{(double)((pos[0]%im->width)-im->width/2), (double)((im->height-pos[0]/im->width)-im->height/2)});
                    if(i == RED){
                        maebot_pose_t msg;
                        msg.x = state->maebot_list[i].curr_pos.x;
                        msg.y = state->maebot_list[i].curr_pos.y;
                        msg.theta = 0.0;
                        state->lcm.publish("MAEBOT_IMAGE_POS_RED",&msg);
                    }
                    else if(i == BLUE){
                        maebot_pose_t msg;
                        msg.x = state->maebot_list[i].curr_pos.x;
                        msg.y = state->maebot_list[i].curr_pos.y;
                        msg.theta = 0.0;
                        state->lcm.publish("MAEBOT_IMAGE_POS_BLUE",&msg);
                    }
                    else if(i == GREEN){
                        maebot_pose_t msg;
                        msg.x = state->maebot_list[i].curr_pos.x;
                        msg.y = state->maebot_list[i].curr_pos.y;
                        msg.theta = 0.0;
                        state->lcm.publish("MAEBOT_IMAGE_POS_GREEN",&msg);
                    }
                }
            }
            // draw maebot position on each window
            for(int i=0;i<NUM_MAEBOT;++i){
                if(state->maebot_list[i].curr_pos.x == DBL_MAX || state->maebot_list[i].curr_pos.y == DBL_MAX){
                    continue;
                }   
                eecs467::Point<double> curr_pos_im_coord = state->calibration.world_to_image_translate(state->maebot_list[i].curr_pos);
                eecs467::Point<double> curr_pos_og_coord = state->calibration.world_to_og_translate(state->maebot_list[i].curr_pos);
                //std::cout<<"curr_pos"<<state->maebot_list[i].curr_pos.x<<" "<<state->maebot_list[i].curr_pos.y<<std::endl;
                // draw to image
                state->im_processor.draw_circle(im,curr_pos_im_coord.x+im->width/2,im->height-(curr_pos_im_coord.y+im->height/2),10.0,state->maebot_list[i].disp_color);
                // draw to occupancy grid
                int npoints = 1;
                float points[3] = {(float)curr_pos_og_coord.x, (float)curr_pos_og_coord.y, 2.0};
                vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                vx_buffer_add_back(og_buf, vxo_points(verts, npoints, vxo_points_style(vx_red, 2.0f)));
            }
        }  

        // if click on maebot selection button, add point to waypoints (from camera image)
        eecs467::Point<double> camera_click_point = state->camera_vx.get_click_point();
        if(camera_click_point.x != -1 && camera_click_point.y!= -1){
            if(state->maebot_curr_selected != NONE){
                eecs467::Point<double> click_world_coord = state->calibration.image_to_world_translate({camera_click_point.x,camera_click_point.y});
                //if click out of boundary
                if(click_world_coord.x<-0.875 || click_world_coord.x>0.875 || click_world_coord.y<-1.5   || click_world_coord.y>1.5){
                    std::cout<<"click out of bound"<<std::endl;
                }
                //if click inside the obstacle
                else if((click_world_coord.x>-0.375 && click_world_coord.x<0.375 && click_world_coord.y<1 && click_world_coord.y>0.25) ||
                        (click_world_coord.x>-0.375 && click_world_coord.x<0.375 && click_world_coord.y<-0.25 && click_world_coord.y>-1)){
                    std::cout<<"click out of bound"<<std::endl;
                }
                //std::cout<<"click point at "<<camera_click_point.x<<" "<<camera_click_point.y<<std::endl;
                //std::cout<<"translated at" << click_world_coord.x <<" "<<click_world_coord.y<<std::endl;
                else{
                    state->maebot_list[state->maebot_curr_selected].waypoints.push_back(click_world_coord);
                    printf("Camera frame: added a way point at: %6.3f %6.3f\n",click_world_coord.x, click_world_coord.y);
                }
            }
        }

        // if click on maebot selection button, add point to waypoints (from occupancy grid)
        eecs467::Point<double> og_click_point = state->occupancy_grid_vx.get_click_point();
        if(og_click_point.x != -1 && og_click_point.y!= -1){
            if(state->maebot_curr_selected != NONE){
                eecs467::Point<double> click_world_coord = state->calibration.og_to_world_translate({og_click_point.x,og_click_point.y});
                //if click out of boundary
                if(click_world_coord.x<-0.875 || click_world_coord.x>0.875 || click_world_coord.y<-1.5   || click_world_coord.y>1.5){
                    std::cout<<"click out of bound"<<std::endl;
                }
                //if click inside the obstacle
                else if((click_world_coord.x>-0.375 && click_world_coord.x<0.375 && click_world_coord.y<1 && click_world_coord.y>0.25) ||
                        (click_world_coord.x>-0.375 && click_world_coord.x<0.375 && click_world_coord.y<-0.25 && click_world_coord.y>-1)){
                    std::cout<<"click out of bound"<<std::endl;
                }
                else{
                    state->maebot_list[state->maebot_curr_selected].waypoints.push_back(click_world_coord);
                    printf("Occupancy Grid frame: added a way point at: %6.3f %6.3f\n",click_world_coord.x,click_world_coord.y);
                }
            }
        }

        vx_object_t *vt = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,"<<center,#000000,serif>>Current Maebot");
        vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-580,30),vxo_mat_scale(0.8),vt)));

        // if maebot selected
        if(state->maebot_curr_selected != NONE){
            //display in the control plane
            if(state->maebot_curr_selected == RED){
                // display maebot text
                vx_object_t *vm = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,"<<center,#ff0000,serif>>RED");
                vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-580,-30),vxo_mat_scale(0.8),vm)));
            } 
            uint waypoints_size = state->maebot_list[state->maebot_curr_selected].waypoints.size();
            if(waypoints_size > 19){
                waypoints_size = 19;
            }
            for(uint i=0;i<waypoints_size;++i){
                char x[31];
                char y[31];
                snprintf(x,31,"<<center,#000000,serif>>%f",state->maebot_list[state->maebot_curr_selected].waypoints[i].x);
                snprintf(y,31,"<<center,#000000,serif>>%f",state->maebot_list[state->maebot_curr_selected].waypoints[i].y); 
                vx_object_t *vwx = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,x);
                vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-490+(int)i*60,30),vxo_mat_scale(0.8),vwx)));
                vx_object_t *vwy = vxo_text_create(VXO_TEXT_ANCHOR_CENTER,y);
                vx_buffer_add_back(control_buf, vxo_pix_coords(VX_ORIGIN_CENTER,vxo_chain(vxo_mat_translate2(-490+(int)i*60,-30),vxo_mat_scale(0.8),vwy)));
            }

            //display the waypoints in the image
            if(!state->maebot_list[state->maebot_curr_selected].waypoints.empty()){
                int npoints = state->maebot_list[state->maebot_curr_selected].waypoints.size();
                float points[npoints*3];
                for(uint i=0;i<state->maebot_list[state->maebot_curr_selected].waypoints.size();++i){
                    eecs467::Point<double> waypoint_im_coord = state->calibration.world_to_image_translate(state->maebot_list[state->maebot_curr_selected].waypoints[i]);
                    points[i*3+0] = waypoint_im_coord.x;
                    points[i*3+1] = waypoint_im_coord.y;
                    points[i*3+2] = 2.0f;
                    //points[i*3+0] = world_to_im;
                    //points[i*3+1] = world_to_im;
                    //points[i*3+2] = 0;
                }
                vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                vx_buffer_add_back(cam_buf, vxo_points(verts, npoints, vxo_points_style(vx_green, 4.0f)));
            }
            
            //display the best particle in the image
            if(state->maebot_list[state->maebot_curr_selected].particle_pos.x != DBL_MAX 
                    && state->maebot_list[state->maebot_curr_selected].particle_pos.y != DBL_MAX){
                int npoints = 1;
                float points[3];
                eecs467::Point<double> particle_pos_im_coord = state->calibration.world_to_image_translate(state->maebot_list[state->maebot_curr_selected].particle_pos);
                points[0] = particle_pos_im_coord.x;
                points[1] = particle_pos_im_coord.y;
                points[2] = 0;
                vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                vx_buffer_add_back(cam_buf, vxo_points(verts, npoints, vxo_points_style(vx_red, 4.0f)));
            }

            //display the waypoints in the occupancy grid
            if(!state->maebot_list[state->maebot_curr_selected].waypoints.empty()){
                int npoints = state->maebot_list[state->maebot_curr_selected].waypoints.size();
                float points[npoints*3];
                for(uint i=0;i<state->maebot_list[state->maebot_curr_selected].waypoints.size();++i){
                    eecs467::Point<double> waypoint_og_coord = state->calibration.world_to_og_translate(state->maebot_list[state->maebot_curr_selected].waypoints[i]);
                    points[i*3+0] = waypoint_og_coord.x;
                    points[i*3+1] = waypoint_og_coord.y;
                    points[i*3+2] = 2.0f;
                    //points[i*3+0] = world_to_im;
                    //points[i*3+1] = world_to_im;
                    //points[i*3+2] = 0;
                }
                vx_resc_t *verts = vx_resc_copyf(points, npoints*3);
                vx_buffer_add_back(og_buf, vxo_points(verts, npoints, vxo_points_style(vx_green, 4.0f)));
            }

            // display maebot path to next waypoint
            // add to camera
            if(state->maebot_list[state->maebot_curr_selected].curr_dest.x != DBL_MAX && state->maebot_list[state->maebot_curr_selected].curr_dest.y != DBL_MAX){
                state->maebot_list[state->maebot_curr_selected].path = state->nav.pathPlan(state->maebot_list[state->maebot_curr_selected].curr_pos, state->maebot_list[state->maebot_curr_selected].curr_dest);
            }
            if(! state->maebot_list[state->maebot_curr_selected].path.empty()){
                std::vector<float> maebot_path;
                eecs467::Point<double> curr_pose_im = state->calibration.world_to_image_translate(state->maebot_list[state->maebot_curr_selected].curr_pos);
                eecs467::Point<double> path_start_im = state->calibration.world_to_image_translate(state->maebot_list[state->maebot_curr_selected].path.front());
                maebot_path.push_back(curr_pose_im.x);
                maebot_path.push_back(curr_pose_im.y);
                maebot_path.push_back(2.0f);
                maebot_path.push_back(path_start_im.x);
                maebot_path.push_back(path_start_im.y);
                maebot_path.push_back(2.0f);
                for(int i = 0; i < state->maebot_list[state->maebot_curr_selected].path.size()-1; i++)
                {
                    eecs467::Point<double> next_im = state->calibration.world_to_image_translate(state->maebot_list[state->maebot_curr_selected].path[i]);
                    eecs467::Point<double> next_im_next = state->calibration.world_to_image_translate(state->maebot_list[state->maebot_curr_selected].path[i+1]);
                    maebot_path.push_back((float)next_im.x);
                    maebot_path.push_back((float)next_im.y);
                    maebot_path.push_back(2.0f);
                    maebot_path.push_back((float)next_im_next.x);
                    maebot_path.push_back((float)next_im_next.y);
                    maebot_path.push_back(2.0f); 
                }
                vx_resc_t *verts = vx_resc_copyf(&maebot_path[0], maebot_path.size());
                vx_buffer_add_back(cam_buf, vxo_lines(verts, maebot_path.size()/3, GL_LINES, vxo_lines_style(vx_purple, 2.0f)));

                // add to occupancy grid
                maebot_path.clear();
                eecs467::Point<double> curr_pose_og = state->calibration.world_to_og_translate(state->maebot_list[state->maebot_curr_selected].curr_pos);
                eecs467::Point<double> path_start_og = state->calibration.world_to_og_translate(state->maebot_list[state->maebot_curr_selected].path.front());
                maebot_path.push_back(curr_pose_og.x);
                maebot_path.push_back(curr_pose_og.y);
                maebot_path.push_back(2.0f);
                maebot_path.push_back(path_start_og.x);
                maebot_path.push_back(path_start_og.y);
                maebot_path.push_back(2.0f);
                for(int i = 0; i < state->maebot_list[state->maebot_curr_selected].path.size()-1; i++)
                {
                    eecs467::Point<double> next_og = state->calibration.world_to_og_translate(state->maebot_list[state->maebot_curr_selected].path[i]);
                    eecs467::Point<double> next_og_next = state->calibration.world_to_og_translate(state->maebot_list[state->maebot_curr_selected].path[i+1]);
                    maebot_path.push_back((float)next_og.x);
                    maebot_path.push_back((float)next_og.y);
                    maebot_path.push_back(2.0f);
                    maebot_path.push_back((float)next_og_next.x);
                    maebot_path.push_back((float)next_og_next.y);
                    maebot_path.push_back(2.0f); 
                }
                vx_resc_t *verts_og = vx_resc_copyf(&maebot_path[0], maebot_path.size());
                vx_buffer_add_back(og_buf, vxo_lines(verts_og, maebot_path.size()/3, GL_LINES, vxo_lines_style(vx_purple, 2.0f)));
            }
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
        vx_buffer_swap(og_buf);
        vx_buffer_swap(control_buf);
        usleep(1000000/fps);
    }
    return NULL;
}

static void* run_lcm(void *input){
    state_t* state = (state_t*) input;
    while(1){
        state->lcm.handle();
    }
    return NULL;
}

int main(int argc, char ** argv){
    g_type_init();
    if (!g_thread_supported ())
        g_thread_init (NULL);

    // Initialize GTK
    gdk_threads_init ();
    gdk_threads_enter ();
    gtk_init (&argc, &argv);

    vx_global_init ();

    state_t state;
    printf("camera finding\n");
    zarray_t *urls = image_source_enumerate();
    if(0 == zarray_size(urls)){
        printf("No camera found.\n");
        exit(1);
    }
    printf("find camera\n");
    zarray_get(urls,1,&state.camera_url);
    printf("get camera address\n");

    draw(&state);
    pthread_create(&state.animate_thread,NULL,render_loop,&state);
    pthread_create(&state.lcm_thread_pid,NULL,run_lcm,&state);
    
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
    gtk_widget_show (canvas3);
    gtk_box_pack_start (GTK_BOX (vbox), pgui, 0, 0, 0);
    gtk_widget_show (pgui);
    gtk_container_add (GTK_CONTAINER (window3), vbox);
    gtk_widget_show (window3);
    // XXX Show all causes errors!

    gtk_widget_show (vbox);
    g_signal_connect_swapped(G_OBJECT(window3), "destroy", G_CALLBACK(gtk_main_quit), NULL);


    gtk_main(); // Blocks as long as GTK window is open

    gdk_threads_leave();
    vx_gtk_display_source_destroy(appwrap);
    vx_global_destroy();
}
