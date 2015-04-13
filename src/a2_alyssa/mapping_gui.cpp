#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "eecs467_util.h"    // This is where a lot of the internals live

#include "state.hpp"
#include "MagicNumbers.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "OccupancyGridGuiHandler.hpp"

using namespace std;
typedef gui_state state_t;

// It's good form for every application to keep its state in a struct.
// Moved to OccupancyGridHandler.hpp 

// === Parameter listener =================================================
// This function is handed to the parameter gui (via a parameter listener)
// and handles events coming from the parameter gui. The parameter listener
// also holds a void* pointer to "impl", which can point to a struct holding
// state, etc if need be.
    static void
my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
}

    static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    return 0;
}

    static int
key_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key)
{
    return 0;
}

    static int
touch_event (vx_event_handler_t *vh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse)
{
    return 0; // Does nothing
}

//Converts logodds [-128, 127] to a grayscale vaue [0, 255]
static int to_grayscale(int a)
{
    return a + 128;
}

    void *
lcm_handle_thread (void *data)
{
    state_t *state = (state_t*)data;
    while(1) state->lcm->handle();
    return NULL;
}

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state
    void *
animate_thread (void *data)
{
    const int fps = 60;
    state_t *state = (state_t*)data;

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) 
    {
		//cout << "Animate Thread!" << endl;
		image_u32_t *im = image_u32_create(state->grid.widthInCells(), state->grid.heightInCells());
		for(int y = 0; y < im->height; y++)
		{
			for(int x = 0; x < im->width; x++)
			{
				int a = 255; //alpha transparency value.
                //std::cout << "(" << x << "," << y << ")\n";
				int v = 255 - to_grayscale(state->grid.logOdds(x, y));
				im->buf[(im->height-1-y)*im->stride+x] = (a<<24) + (v<<16) + (v<<8) + (v<<0);
			}
		}
        vx_object_t * vo = vxo_image_from_u32(im, VXO_IMAGE_FLIPY, VX_TEX_MIN_FILTER);
        const double scale = 1./im->width;
        //const double scale = 1.0;

        auto mat_scale = vxo_mat_scale3(scale, scale, 1.0);
        //vxo_mat_translate3(-im->width/2., -im->height/2., 0.);

        pthread_mutex_lock(&state->gui_mutex);
        vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "bitmap"),
                            vxo_chain (mat_scale,
                                       vxo_mat_translate3(-im->width/2., -im->height/2., 0.),
                                       vo));// drawing happens here

        //std::cout << "state poses: " << state->poses.size() << std::endl;
        std::vector<float> points;
        for(unsigned int i = 0; i < state->poses.size(); i++)
        {
            /*if(i == state->poses.size() - 1)
            {
                std::cout << "last pose: " << state->poses[i].x << ", " << state->poses[i].y << std::endl;
            }*/
            points.push_back((float)(state->poses[i].x/state->grid.metersPerCell()));
            points.push_back((float)(state->poses[i].y/state->grid.metersPerCell()));
            points.push_back(0);
        }
        
        //std::cout << "true poses: " << state->truePoses.size() << std::endl;
        std::vector<float> truePoints;
        for(unsigned int i = 0; i < state->truePoses.size(); i++)
        {
            /*if(i == state->truePoses.size() - 1)
            {
                std::cout << "last true pose: " << state->truePoses[i].x << ", " << state->truePoses[i].y << std::endl;
            }*/
            truePoints.push_back((float)(state->truePoses[i].x/state->grid.metersPerCell()));
            truePoints.push_back((float)(state->truePoses[i].y/state->grid.metersPerCell()));
            truePoints.push_back(0);
        }

        //std::cout << "particles size: " << state->particles.size() << std::endl;
        std::vector<float> particlePoints;
        for(unsigned int i = 0; i < state->particles.size(); i++)
        {
            //if(i == state->particles.size() - 1)
            //{
            //    std::cout << state->particles[i].x << ", " << state->particles[i].y << std::endl;
            //}
            particlePoints.push_back((float)(state->particles[i].x/state->grid.metersPerCell()));
            particlePoints.push_back((float)(state->particles[i].y/state->grid.metersPerCell()));
            particlePoints.push_back(0);
        }
        //std::cout << "particle point size: " << particlePoints.size() << std::endl;
        vx_resc_t *verts = vx_resc_copyf(&points[0], points.size());
        vx_resc_t *trueVerts = vx_resc_copyf(&truePoints[0], truePoints.size());
        vx_resc_t *particleVerts = vx_resc_copyf(&particlePoints[0], particlePoints.size());

        vx_buffer_t *buff = vx_world_get_buffer(state->vxworld, "points");
        vx_buffer_add_back (buff,
                            vxo_chain (mat_scale,
                                       vxo_points(verts, state->poses.size(), vxo_points_style(vx_green, 4.0f)),
                                       vxo_points(trueVerts, state->truePoses.size(), vxo_points_style(vx_blue, 4.0f)),
                                       vxo_points(particleVerts, state->particles.size(), vxo_points_style(vx_red, 2.0f))));
        
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "bitmap"));
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "points"));
        //vx_buffer_swap (vx_world_get_buffer (state->vxworld, "points2"));
        image_u32_destroy (im);

        //vx_buffer_swap (vx_world_get_buffer (state->vxworld, "axes"));
        pthread_mutex_unlock(&state->gui_mutex);

        usleep (1000000/fps);
    }

    return NULL;
}

//void * 
//receive_lcm_msg (void *data)
//{
//    state_t *state = (state_t *)data;
//
//    while(1)
//    
//    return NULL;
//}

    state_t *
state_create (void)
{
    state_t *state = new state_t;

    state->vxworld = vx_world_create ();
    state->vxeh = new vx_event_handler_t;
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->touch_event = touch_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    state->running = 1;
    state->location_count = 0;

    return state;
}

    void
state_destroy (state_t *state)
{
    if (!state)
        return;

    if (state->vxeh)
        free (state->vxeh);

    if (state->gopt)
        getopt_destroy (state->gopt);

    if (state->pg)
        pg_destroy (state->pg);

    free (state);
}

// This is intended to give you a starting point to work with for any program
// requiring a GUI. This handles all of the GTK and vx setup, allowing you to
// fill in the functionality with your own code.
    int
main (int argc, char *argv[])
{
    eecs467_init (argc, argv);
    state_t *state = state_create ();

    OccupancyGridGuiHandler gui_handler(&state->grid);
    LocationHandler location_handler(state);

    state->lcm = new lcm::LCM;
    pthread_mutex_init(&state->lcm_mutex, NULL);

    state->lcm->subscribe("OCCUPANCY_GRID_GUI_TASK2",
            &OccupancyGridGuiHandler::handleMessage,
            &gui_handler);
    state->lcm->subscribe("MAEBOT_POSE_GUI", 
            &LocationHandler::handlePose,
            &location_handler);
    state->lcm->subscribe("MAEBOT_POSE", 
            &LocationHandler::handleTruePose,
            &location_handler);
    state->lcm->subscribe("MAEBOT_PARTICLE_GUI", 
            &LocationHandler::handleParticles,
            &location_handler);

    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_string (state->gopt, '\0', "url", "", "Camera URL");
    getopt_add_bool   (state->gopt,  'l', "list", 0, "Lists available camera URLs and exit");

    if (!getopt_parse (state->gopt, argc, argv, 1) || getopt_get_bool (state->gopt, "help")) {
        printf ("Usage: %s [--url=CAMERAURL] [other options]\n\n", argv[0]);
        getopt_do_usage (state->gopt);
        exit (EXIT_FAILURE);
    }

    // Set up the imagesource. This looks for a camera url specified on
    // the command line and, if none is found, enumerates a list of all
    // cameras imagesource can find and picks the first url it finds.
    if (strncmp (getopt_get_string (state->gopt, "url"), "", 1)) {
        state->img_url = strdup (getopt_get_string (state->gopt, "url"));
        printf ("URL: %s\n", state->img_url);
    }
    else {
        // No URL specified. Show all available and then use the first
        zarray_t *urls = image_source_enumerate ();
        printf ("Cameras:\n");
        for (int i = 0; i < zarray_size (urls); i++) {
            char *url;
            zarray_get (urls, i, &url);
            printf ("  %3d: %s\n", i, url);
        }

        if (0==zarray_size (urls)) {
            printf ("Found no cameras.\n");
            return -1;
        }

        zarray_get (urls, 0, &state->img_url);
    }

    if (getopt_get_bool (state->gopt, "list")) {
        state_destroy (state);
        exit (EXIT_SUCCESS);
    }

    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.

    // Initialize a parameter gui
    state->pg = pg_create ();

    parameter_listener_t *my_listener = new parameter_listener_t;
    my_listener->impl = state;
    my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);

    // Launch our worker threads
    pthread_create (&state->animate_thread, NULL, animate_thread, (void*)state);
    pthread_create (&state->lcm_thread, NULL, lcm_handle_thread, (void*)state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, SCREEN_WIDTH, SCREEN_HEIGHT);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);


    // Cleanup
    free (my_listener);
    state_destroy (state);
    vx_global_destroy ();
}
