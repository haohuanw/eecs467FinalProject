#ifndef VX_STATE_T
#define VX_STATE_T
#include "math/point.hpp"
#include <string>
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"

class vx_state_t{
    private:
        eecs467::Point<double>   click_point;
    public:
        std::string             vxname;
        vx_application_t        vxapp;
        vx_world_t              *vxworld;
        zhash_t                 *layers;
        vx_gtk_display_source_t *appwrap;
        vx_event_handler_t      *vxeh;
        vx_mouse_event_t        last_mouse_event;
        pthread_mutex_t         mutex;
    public:
        vx_state_t();
        vx_state_t(std::string name);        
        ~vx_state_t();        
        eecs467::Point<double> get_click_point();
        static void display_finished(vx_application_t * app, vx_display_t * disp)
        {
            vx_state_t * state = (vx_state_t*)app->impl;
            pthread_mutex_lock(&state->mutex);

            vx_layer_t * layer = NULL;

            // store a reference to the world and layer that we associate with each vx_display_t
            zhash_remove(state->layers, &disp, NULL, &layer);

            vx_layer_destroy(layer);

            pthread_mutex_unlock(&state->mutex);
        }

        static void display_started(vx_application_t * app, vx_display_t * disp)
        {
            vx_state_t * state = (vx_state_t*)app->impl;

            vx_layer_t * layer = vx_layer_create(state->vxworld);
            vx_layer_set_display(layer, disp);
            //important setting for the handler
            if(state->vxeh != NULL){
                vx_layer_add_event_handler(layer,state->vxeh);
            }
            pthread_mutex_lock(&state->mutex);
            // store a reference to the world and layer that we associate with each vx_display_t
            zhash_put(state->layers, &disp, &layer, NULL, NULL);
            pthread_mutex_unlock(&state->mutex);
        }


        static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
        {
            vx_state_t *state = (vx_state_t*)vxeh->impl;
            pthread_mutex_lock(&state->mutex);
            // vx_camera_pos_t contains camera location, field of view, etc
            // vx_mouse_event_t contains scroll, x/y, and button click events
            if ((mouse->button_mask & VX_BUTTON1_MASK) &&
                    !(state->last_mouse_event.button_mask & VX_BUTTON1_MASK)) {
                //state->click_point.x = mouse->x;
                //state->click_point.y = mouse->y;
                vx_ray3_t ray;
                vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);
                double ground[3];
                vx_ray3_intersect_xy (&ray, 0, ground);
                printf ("%s : Mouse clicked at coords: [%6.3f, %6.3f]\n",
                        state->vxname.c_str(),ground[0], ground[1]);
                state->click_point.x = ground[0];
                state->click_point.y = ground[1];
            }
            state->last_mouse_event = *mouse;
            pthread_mutex_unlock(&state->mutex);
            return 0;
        }

        static int key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
        { 
            return 0;
        }

        static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
        {
            return 0; // Does nothing
        }

        /*static int mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse);
        static int key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key);
        static int touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse);
        static void display_finished(vx_application_t * app, vx_display_t * disp);
        static void display_started(vx_application_t * app, vx_display_t * disp);*/

};



#endif
