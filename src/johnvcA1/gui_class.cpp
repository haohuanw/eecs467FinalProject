#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <string>
#include <iostream>
#include <vector>
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
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/maebot_pose_t.hpp"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "lcmtypes/maebot_occupancy_grid_t.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "math/point.hpp"
#include "math/angle_functions.hpp"
#include "mapping/occupancy_grid.hpp"
//#include "lcm-cpp.hpp"
#include "eecs467_util.h"    // This is where a lot of the internals live

using namespace std;

lcm::LCM lcm_inst;

lcm::LCM lcm_lidar;


eecs467::OccupancyGrid OG;

typedef struct state state_t;
struct state {
    bool running;

    pthread_mutex_t lcm_mutex;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;

    // threads
    pthread_t animate_thread;
    pthread_t handlelcm;
    pthread_t draw_thread; 
    pthread_t lidar_thread;

    // for accessing the arrays
    pthread_mutex_t mutex;

};

state_t* state;

class gui_class{
public:
  bool running;
  getopt_t *gopt;
  parameter_gui_t *pg;
  
  //vx variables
  /*
  vx_application_t vxapp;
  vx_world_t *vx_world;
  vx_event_handler_t *vxeh;
  */
  //motion variables
  long timestamp_0, timestamp_0_2;
  long timestamp_1, timestamp_1_2;
  float x, x_2;
  float y, y_2;
  float theta, theta_2, w;
  float l_theta;
  int num_ranges;
  int counter;
  int draw_counter;
  int draw_counter2;
  vector<float> ranges;
  vector<float> thetas;
  vector<long> times;
  vector<float> intensities;

  //occ grid var
  int64_t o_time;
  float origin_x;
  float origin_y;
  float meters_per_cell;
  int32_t width;
  int32_t height;
  int32_t num_cells;
  vector<int8_t> cells;
  
  
  //threads
  pthread_t animate_thread;
  pthread_t lcm_thread;

  //mutexes
  pthread_mutex_t lcm_mutex;

  //GUI CLASS CONSTRUCTOR
  gui_class(){
    gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "--help", 0, "HALP");

    timestamp_0 = timestamp_1 = 0;
    x = y = theta = counter = draw_counter = draw_counter2 = 0;

    //state_t *state = calloc (1, sizeof(*state));
    

    //magic happens here, just go with it
    /*
    vx_world = vx_world_create ();
    vxapp.display_started = eecs467_default_display_started;
    vxapp.display_finished = eecs467_default_display_finished;
    vxapp.impl = eecs467_default_implementation_create (vx_world, vxeh);
    */
    running = true;

  }

  //GUI CLASS DESTRUCTOR
  ~gui_class(){
    if (gopt)
        getopt_destroy (gopt);

    if (pg)
        pg_destroy (pg);
  }

  //POSE HANDLER
  void maebot_pose_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_pose_t* msg)
  {
    
    float dist_points[6];
    int dist_npoints = 2;
    //  int res = system ("clear");
    //    if (res)
    //printf ("system clear failed\n");
    if(timestamp_1 != msg->utime){
      timestamp_0 = timestamp_1;
      timestamp_1 = msg->utime;
    }
	//cout << "hello" << endl;
   // cout<<msg->x<<endl;
    theta = msg->theta;
    timestamp_1 = msg->utime;
    

    dist_points[0] = x;
    dist_points[1] = y;
    dist_points[2] = 0;
    dist_points[3] = msg->x;
    dist_points[4] = msg->y;
    dist_points[5] = 0;

    x = msg->x;
    y = msg->y;
    vx_resc_t *dist_verts = vx_resc_copyf(dist_points, dist_npoints*3);

    char temp[100000];
    sprintf(temp,"position_scan_%i",draw_counter++);
    vx_buffer_add_back(vx_world_get_buffer (state->vxworld, temp), vxo_lines(dist_verts, dist_npoints, GL_LINES,vxo_points_style(vx_red, 2.0f)));         
    vx_buffer_swap (vx_world_get_buffer (state->vxworld, temp));
    


   }

  void task2_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_pose_t* msg)
  {
    float dist_points[6];
    int dist_npoints = 2;
    //  int res = system ("clear");
    //    if (res)
    //printf ("system clear failed\n");
    if(timestamp_1_2 != msg->utime){
      timestamp_0_2 = timestamp_1_2;
      timestamp_1_2 = msg->utime;
    }
	//cout << "hello" << endl;
   // cout<<msg->x<<endl;
    w = eecs467::wrap_to_pi(msg->theta - theta_2)/( timestamp_1_2 - timestamp_0_2);

    theta_2 = msg->theta;
    timestamp_1_2 = msg->utime;

    dist_points[0] = x_2;
    dist_points[1] = y_2;
    dist_points[2] = 0;
    dist_points[3] = msg->x;
    dist_points[4] = msg->y;
    dist_points[5] = 0;

    x_2 =msg->x;
    y_2 = msg->y;
    vx_resc_t *dist_verts = vx_resc_copyf(dist_points, dist_npoints*3);

    char temp[100000];
    sprintf(temp,"position_scan_%i",draw_counter++);
    vx_buffer_add_back(vx_world_get_buffer (state->vxworld, temp), vxo_lines(dist_verts, dist_npoints, GL_LINES,vxo_points_style(vx_green, 2.0f)));         
        vx_buffer_swap (vx_world_get_buffer (state->vxworld, temp));



   }

 void laser_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_laser_scan_t* msg)
{
  num_ranges = msg->num_ranges;
  ranges = msg->ranges;
  thetas = msg->thetas;
  times = msg->times;
  intensities = msg->intensities;
  //cout << "num ranges: " << msg->num_ranges << endl;


////////////////////// LASER SCANNING /////////////////////////////
	float laser_points[6*num_ranges];
	int laser_npoints = 2*num_ranges;
        float theta_err = 0;

	for(int i = 0; i<msg->num_ranges; i++)
	{
	  theta_err = w*float(msg->times[i] - timestamp_0_2);
	  laser_points[6*i + 0] = x_2;
	  laser_points[6*i + 1] = y_2;
	  laser_points[6*i + 2] = 0;
	  laser_points[6*i + 3] = x_2 + msg->ranges[i]*cosf(eecs467::wrap_to_pi(eecs467::wrap_to_pi(theta + theta_err) - msg->thetas[i]));
	  laser_points[6*i + 4] = y_2 + msg->ranges[i]*sinf( eecs467::wrap_to_pi(eecs467::wrap_to_pi(theta + theta_err) - msg->thetas[i]));
	  laser_points[6*i + 5] = 0;
	  //laser_npoints = 2;

  
 	}
	//vx_resc_t *laser_verts = vx_resc_copyf(laser_points, laser_npoints*3);
  	//vx_buffer_add_back(vx_world_get_buffer (state->vxworld, "Laser_scan"), vxo_lines(laser_verts, laser_npoints, GL_LINES, vxo_points_style(vx_blue, 2.0f)));  
  	//vx_buffer_swap (vx_world_get_buffer (state->vxworld, "Laser_scan"));


///////////////////////////////////////////////////////////////////
}

uint8_t to_grayscale(int8_t val)
{
  int x = (int) val;
  x+=128;
  x = 255-x;
  uint8_t y = (uint8_t) x;
  return y;
}

 void OGM_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_occupancy_grid_t* msg)
{
  OG.fromLCM(*msg);

  image_u8_t *img = image_u8_create(msg->width, msg->height);
  for(int y = 0; y< msg->height-1; y++)
    {
      for(int x = 0; x < msg->width-1; x++)
	{
	  img->buf[y*img->stride + x] = to_grayscale(OG(x,y));
	}
    }

  vx_object_t *obj = vxo_chain( vxo_mat_translate3(-msg->meters_per_cell*msg->width/2, -msg->meters_per_cell*msg->height/2, -0.01),vxo_mat_scale(msg->meters_per_cell), vxo_image_from_u8(img, 0 , 0));
  vx_buffer_add_back( vx_world_get_buffer(state->vxworld, "OG_map"), obj );
  vx_buffer_swap( vx_world_get_buffer(state->vxworld, "OG_map"));
  
  //draw the image somehow
}

};
//end of class


//LCM COMMUNICATION THREAD
  void *lcm_comm(void *input){

    //    lcm_t *lcm = (lcm_t *) lcm_inst;
   while (true){
      lcm_inst.handle();
    }

    return NULL;
 }
void* lidar_comm(void *input)
{
    while(1)
    {
   	lcm_lidar.handle();
    }
    return NULL;
}

static void
my_param_changed (parameter_listener_t *pl, parameter_gui_t *pg, const char *name)
{
    if (0==strcmp ("sl1", name))
        printf ("sl1 = %f\n", pg_gd (pg, name));
    else if (0==strcmp ("sl2", name))
        printf ("sl2 = %d\n", pg_gi (pg, name));
    else if (0==strcmp ("cb1", name) || 0==strcmp ("cb2", name))
        printf ("%s = %d\n", name, pg_gb (pg, name));
    else
        printf ("%s changed\n", name);
}

static int
mouse_event (vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse)
{
    state = static_cast<state_t*>(vxeh->impl);

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

state_t *
state_create (void)
{
    state_t *state = static_cast<state_t*>(calloc (1, sizeof(*state)));

    state->vxworld = vx_world_create ();
    state->vxeh = static_cast<vx_event_handler_t*>(calloc (1, sizeof(*state->vxeh)));
    state->vxeh->key_event = key_event;
    state->vxeh->mouse_event = mouse_event;
    state->vxeh->touch_event = touch_event;
    state->vxeh->dispatch_order = 100;
    state->vxeh->impl = state; // this gets passed to events, so store useful struct here!

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

int main(int argc, char *argv[]){
    eecs467_init (argc, argv);
    state = state_create ();
    OG.reset();
    //state->lcm = lcm_create(NULL);

  gui_class G;
  //create an LCM instance
  //subscribeFunction ties maebot_pose_handler to the MAEBOT_POSE channel
  //  lcm::LCM lcm_inst;
  vx_remote_display_source_t *cxn = vx_remote_display_source_create (&state->vxapp);

  // Initialize a parameter gui
  state->pg = pg_create ();
  pg_add_double_slider (state->pg, "sl1", "Slider 1", 0, 100, 10);
  pg_add_int_slider    (state->pg, "sl2", "Slider 2", 0, 100, 10);
  pg_add_check_boxes (state->pg,
                        "cb1", "Check Box 1", 0,
                        "cb2", "Check Box 2", 1,
                        NULL);
  pg_add_buttons (state->pg,
                    "but1", "Button 1",
                    "but2", "Button 2",
                    "but3", "Button 3",
                    NULL);

  parameter_listener_t *my_listener = static_cast<parameter_listener_t*>(calloc (1, sizeof(*my_listener)));
  my_listener->impl = state;
  my_listener->param_changed = my_param_changed;
  pg_add_listener (state->pg, my_listener);


  lcm_inst.subscribe("MAEBOT_POSE", &gui_class::maebot_pose_handler, &G);
  lcm_inst.subscribe("MAEBOT_LASER_SCAN", &gui_class::laser_handler, &G);
  lcm_inst.subscribe("MAEBOT_OCCUPANCY_GRID", &gui_class::OGM_handler, &G);
  lcm_inst.subscribe("MAEBOT_TASK2", &gui_class::task2_handler, &G);
 
  //initiate LCM thread
  pthread_t lcm_comm_th;
  pthread_create(&lcm_comm_th, NULL, lcm_comm , NULL);
  pthread_t lidar_comm_th;
  pthread_create(&lidar_comm_th, NULL, lidar_comm, NULL);



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
  return 0;
  }

