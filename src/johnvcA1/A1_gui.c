#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
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

//maebot message subcription
#include "common/timestamp.h"
#include "lcmtypes/maebot_motor_command_t.h"
#include "lcmtypes/maebot_motor_feedback_t.h"
#include "lcmtypes/maebot_sensor_data_t.h"
#include "lcmtypes/maebot_laser_scan_t.h"

#include "eecs467_util.h"    // This is where a lot of the internals live

// It's good form for every application to keep its state in a struct.
typedef struct state state_t;
struct state {
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
  //vx_event_handler_t *vxeh; // for getting mouse, key, and touch events

    // threads
    pthread_t animate_thread;

    // for accessing the arrays
    pthread_mutex_t mutex;
};

bool run;

// === Your code goes here ================================================
// The render loop handles your visualization updates. It is the function run
// by the animate_thread. It periodically renders the contents on the
// vx world contained by state

//needed variables/mutexes
pthread_mutex_t run_mutex;

/*LCM communication thread*/
void *run_feedback(void *input)
{
  lcm_t *lcm = (lcm_t *) input;
  pthread_mutex_lock(&run_mutex);
  while (state.running){
    pthread_mutex_unlock(&run_mutex);
    lcm_handle (lcm);
    pthread_mutex_lock(&run_mutex);
  }
  pthread_mutex_unlock(&run_mutex);
  return NULL;
}

/*for parsing the motor feedback data*/
static void
motor_feedback_handler (const lcm_recv_buf_t *rbuf, const char *channel,
const maebot_motor_feedback_t *msg, void *user)
{
  int res = system ("clear");
  if (res)
    printf ("system clear failed\n");
  int left_ticks = msg->encoder_left_ticks;
  int right_ticks = msg->encoder_right_ticks;
  motor_feedback_timestamp = msg->utime;
  fprintf(fp_data, "%lld %d %d\n", msg->utime,left_ticks, right_ticks);
}

/*for pasrsing lidar data from LCM*/
static void
maebot_laser_scan_handler(const lcm_recv_buf_t *rbuf, const char *channel,
			  const maebot_laser_scan_t *msg, void *user)
{
  int res = system ("clear");
  if (res)
    printf ("system clear failed\n");
  //fprintf(fp_lcm, "Subscribed to channel: MAEBOT_LASER_SCAN\n");
  int i;
  //fprintf(fp_lcm, "Num Ranges: %d\n", msg->num_ranges);
  fprintf(fp_lcm,"%lld\n",motor_feedback_timestamp);
  fprintf(fp_lcm,"%d\n",msg->num_ranges);
  for(i = 0; i < msg->num_ranges; i++)
    {
      fprintf(fp_lcm, "%f %f\n", msg->ranges[i],msg->thetas[i]);
    }
  fprintf(fp_lcm, "\n");
}

void *
animate_thread (void *data)
{
    const int fps = 60;
    state_t *state = data;

    // Continue running until we are signaled otherwise. This happens
    // when the window is closed/Ctrl+C is received.
    while (state->running) {
      run = true;
        // Example rendering of vx primitives
        double rad = (vx_util_mtime () % 5000) * 2. * M_PI / 5e3;   // [ 0, 2PI]
        double osc = ((vx_util_mtime () % 5000) / 5e3) * 2. - 1;    // [-1, 1]

        // Creates a blue box and applies a series of rigid body transformations
        // to it. A vxo_chain applies its arguments sequentially. In this case,
        // then, we rotate our coordinate frame by rad radians, as determined
        // by the current time above. Then, the origin of our coordinate frame
        // is translated 0 meters along its X-axis and 0.5 meters along its
        // Y-axis. Finally, a 0.1 x 0.1 x 0.1 cube (or box) is rendered centered at the
        // origin, and is rendered with the blue mesh style, meaning it has
        // solid, blue sides.
        vx_object_t *vxo_sphere = vxo_chain (vxo_mat_rotate_z (rad),
                                             vxo_mat_translate2 (0, 0.5),
                                             vxo_mat_scale (0.1),
                                             vxo_sphere (vxo_mesh_style (vx_blue)));

        // Then, we add this object to a buffer awaiting a render order
        vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "rot-sphere"), vxo_sphere);

        // Now we will render a red box that translates back and forth. This
        // time, there is no rotation of our coordinate frame, so the box will
        // just slide back and forth along the X axis. This box is rendered
        // with a red line style, meaning it will appear as a red wireframe,
        // in this case, with lines 2 px wide at a scale of 0.1 x 0.1 x 0.1.
        vx_object_t *vxo_square = vxo_chain (vxo_mat_translate2 (osc, 0),
                                             vxo_mat_scale (0.1),
                                             vxo_box (vxo_lines_style (vx_red, 2)));

        // We add this object to a different buffer so it may be rendered
        // separately if desired
        vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "osc-square"), vxo_square);


        // Draw a default set of coordinate axes
        vx_object_t *vxo_axe = vxo_chain (vxo_mat_scale (0.1), // 10 cm axes
                                          vxo_axes ());
        vx_buffer_add_back (vx_world_get_buffer (state->vxworld, "axes"), vxo_axe);


        // Now, we update both buffers
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "rot-sphere"));
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "osc-square"));
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, "axes"));

        usleep (1000000/fps);
    }

    return NULL;
}

state_t *
state_create (void)
{
    state_t *state = calloc (1, sizeof(*state));

    state->vxworld = vx_world_create ();
    //state->vxeh = calloc (1, sizeof(*state->vxeh));
    //state->vxeh->key_event = key_event;
    //state->vxeh->mouse_event = mouse_event;
    //state->vxeh->touch_event = touch_event;
    //state->vxeh->dispatch_order = 100;
    //state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

    state->vxapp.display_started = eecs467_default_display_started;
    state->vxapp.display_finished = eecs467_default_display_finished;
    state->vxapp.impl = eecs467_default_implementation_create (state->vxworld, state->vxeh);

    state->running = 1;

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

    /*SUBSCRIBE TO LCM DATA STREAMS*/
    maebot_motor_feedback_t_subscribe (lcm,"MAEBOT_MOTOR_FEEDBACK",motor_feedback_handler, NULL);
    maebot_laser_scan_t_subscribe(lcm_lidar, "MAEBOT_LASER_SCAN", maebot_laser_scan_handler, NULL);


    // Parse arguments from the command line, showing the help
    // screen if required
    state->gopt = getopt_create ();
    getopt_add_bool   (state->gopt,  'h', "help", 0, "Show help");
    getopt_add_bool   (state->gopt,  'l', "list", 0, "Lists available camera URLs and exit");
    
    // Initialize this application as a remote display source. This allows
    // you to use remote displays to render your visualization. Also starts up
    // the animation thread, in which a render loop is run to update your display.
    vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

    // Initialize a parameter gui
    state->pg = pg_create ();
    
    parameter_listener_t *my_listener = calloc (1, sizeof(*my_listener));
    my_listener->impl = state;
    //my_listener->param_changed = my_param_changed;
    pg_add_listener (state->pg, my_listener);
    
    // launch our worker threads

    //launch the animate thread 
    pthread_create (&state->animate_thread, NULL, animate_thread, state);

    // This is the main loop
    eecs467_gui_run (&state->vxapp, state->pg, 1024, 768);

    // Quit when GTK closes
    state->running = 0;
    pthread_join (state->animate_thread, NULL);

    // Cleanup
    free (my_listener);
    state_destroy (state);
    vx_remote_display_source_destroy (cxn);
    vx_global_destroy ();
}
