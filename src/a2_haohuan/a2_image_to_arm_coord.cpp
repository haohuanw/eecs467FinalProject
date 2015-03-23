#include <thread>
#include <pthread.h>
#include <fstream>
#include <algorithm>

#include <gtk/gtk.h>
#include <lcm/lcm-cpp.hpp>
#include <gsl/gsl_linalg.h>

#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/gtk/vx_gtk_display_source.h"
#include "vx/vxo_drawables.h"
#include "common/zarray.h"
#include "common/getopt.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"
#include "math/point.hpp"
#include "math/matd.h"

#include "hsv.hpp"
#include "a2_inv_kin.hpp"
#include "image_processor.hpp"
using namespace std;

typedef struct state state_t;
struct state
{
  getopt_t *gopt;

  lcm_t *lcm;
  const char *command_channel;
  const char *status_channel;

  char *camera_url;
  char *pic_url;
  bool usePic;
  image_processor im_proc;
  vx_application_t vxapp;
  vx_world_t *vxworld;
  zhash_t *layers;
  vx_event_handler_t *vxeh;
  vx_mouse_event_t last_mouse_event;
  vx_gtk_display_source_t *appwrap;

  max_min_hsv cyan_hsv;  
  eecs467::Point<float> corner_coords[2];
  eecs467::Point<float> click_point;
  eecs467::Point<double> arm_coords;
  int im_width;
  int im_height;
  //eecs467::Point<double> arm_coords_inCam;
  FILE *printfile;
  FILE *nine_squares;
  int nine_squares_count;
  bool has_tx;
  matd_t *tx_mat;

  thread status_thread;
  thread command_thread;
  thread animate_thread;

  pthread_mutex_t mutex;
  pthread_mutex_t data_mutex;
};
state_t state;

static void status_handler(const lcm_recv_buf_t *rbuf,
			   const char *channel,
			   const dynamixel_status_list_t *msg,
			   void *user){
  //do stuff
  vector<double> angles(msg->len);
  for(int i=0; i<msg->len; ++i){
    angles[i] = msg->statuses[i].position_radians;
    //printf("[id%d]=%6.3f ", i, msg->statuses[i].position_radians);
  }
  //cout << endl;
  double R, r1, r2, r3;
  r1 = d2 * fsin(angles[1]);
  r2 = d3 * fcos(angles[2] - ((M_PI/2.0) - angles[1]));
  r3 = (d4 + 0.065) * fcos(angles[3] + (angles[2] - ((M_PI/2.0) - angles[1])));
  R = r1 + r2 + r3;
  double theta = angles[0];
  //cout << "R: " << R << " theta: " << theta << endl;
  double x = R * fcos(theta);
  double y = R * fsin(theta);
  //cout << "X: " << x << " Y: " << y << endl;
  pthread_mutex_lock(&state.data_mutex);
  state.arm_coords.x = x;
  state.arm_coords.y = y;
  pthread_mutex_unlock(&state.data_mutex);
}

void status_loop(){
  dynamixel_status_list_t_subscribe (state.lcm,
				     state.status_channel,
				     status_handler,
				     &state);
  while(1){
    lcm_handle(state.lcm);
  }
}

void command_loop(){
  dynamixel_command_list_t cmds;
  cmds.len = NUM_SERVOS;
  cmds.commands = (dynamixel_command_t*) calloc(NUM_SERVOS, sizeof(dynamixel_command_t));
  while(1){
    for(int id = 0; id < NUM_SERVOS; ++id){
      cmds.commands[id].utime = utime_now();
      cmds.commands[id].position_radians = 0.0;
      cmds.commands[id].speed = 0.0;
      cmds.commands[id].max_torque = 0.0;
    }
    dynamixel_command_list_t_publish (state.lcm, state.command_channel, &cmds);
  }
  free(cmds.commands);

  
}

void render_loop(){
  int fps = 60;
  image_source_t *isrc = NULL;
  if(!state.usePic){
    isrc = image_source_open(state.camera_url);
    if(isrc == NULL){
      cout << "Error opening device" << endl;
      exit(1);
    }
    else
      isrc->start(isrc);
  }
  pthread_mutex_lock(&state.data_mutex);
  vx_buffer_t *buf = vx_world_get_buffer(state.vxworld,"image");
  std::vector<int> cyan_center_list;
  image_u32_t *im;
  if(state.usePic){
    im = image_u32_create_from_pnm(state.pic_url); 
    state.im_width = im->width;
    state.im_height = im->height;
  }
  else if(isrc != NULL){
    image_source_data_t *frmd = (image_source_data_t*) calloc(1,sizeof(*frmd));
    int res = isrc->get_frame(isrc,frmd);
    if(res < 0){
      printf("get_frame fail\n");
    }
    else{
      im = image_convert_u32(frmd);
      state.im_width = im->width;
      state.im_height = im->height;
    }
    fflush(stdout);
    isrc->release_frame(isrc,frmd);
  }
  if((state.corner_coords[0].x != -1) && (state.corner_coords[0].y != -1) && (state.corner_coords[1].x != -1) && (state.corner_coords[1].y != -1)){  
    state.im_proc.image_masking(im, state.corner_coords[0].x, state.corner_coords[1].x, state.corner_coords[0].y, state.corner_coords[1].y);
    cyan_center_list = state.im_proc.blob_detection(im, state.corner_coords[0].x, state.corner_coords[1].x, state.corner_coords[0].y, state.corner_coords[1].y, state.cyan_hsv);
  }
  if(cyan_center_list.size() == 4){
    sort(cyan_center_list.begin(), cyan_center_list.end());
    //FILE *cfile = fopen("../calibration/cyan_centers.txt","w");
    ofstream cfile("../calibration/cyan_centers.txt");
    for(int i=0; i<cyan_center_list.size(); ++i){
      int y = (cyan_center_list[i]) / state.im_width;
      int x = (cyan_center_list[i]) % state.im_width;
      state.im_proc.draw_circle(im, x, y, 20.0, 0xffffff00);
      cout << "Centroid[" << i << "]: (" << x << ", " << y << ")" << endl;
      //fprintf(cfile, "%f %f\n", x, y);
      cfile << x << ' ' << y << endl;
    }
    //fclose(cfile);
  }
  else{
    cout << "Initial cyan centroids != 4" << endl;
    exit(1);
  }
    
  if(im != NULL){
    vx_object_t *vim = vxo_image_from_u32(im,
					  VXO_IMAGE_FLIPY,
					  VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
    //use pix coords to make a fix image
    vx_buffer_add_back (buf,vxo_chain (
				       vxo_mat_translate3 (-state.im_width/2., -state.im_height/2., 0.),
				       vim));
      
      
      
  }
  image_u32_destroy(im);
  pthread_mutex_unlock(&state.data_mutex);
  vx_buffer_swap(buf);
  usleep(1000000/fps);

  while(1){
    pthread_mutex_lock(&state.data_mutex);
    if(state.has_tx){
      image_u32_t *im;
      if(isrc != NULL){
	image_source_data_t *frmd = (image_source_data_t*) calloc(1,sizeof(*frmd));
	int res = isrc->get_frame(isrc,frmd);
	if(res < 0){
	  printf("get_frame fail\n");
	}
	else{
	  im = image_convert_u32(frmd);
	  state.im_width = im->width;
	  state.im_height = im->height;
	}
	fflush(stdout);
	isrc->release_frame(isrc,frmd);
      }
      if((state.corner_coords[0].x != -1) && (state.corner_coords[0].y != -1) && (state.corner_coords[1].x != -1) && (state.corner_coords[1].y != -1)){  
	state.im_proc.image_masking(im, state.corner_coords[0].x, state.corner_coords[1].x, state.corner_coords[0].y, state.corner_coords[1].y);
	cyan_center_list = state.im_proc.blob_detection(im, state.corner_coords[0].x, state.corner_coords[1].x, state.corner_coords[0].y, state.corner_coords[1].y, state.cyan_hsv);
      }
      if(!cyan_center_list.empty()){
	for(int i=0; i<cyan_center_list.size(); ++i){
	  int y = (cyan_center_list[i]) / state.im_width;
	  int x = (cyan_center_list[i]) % state.im_width;
	  state.im_proc.draw_circle(im, x, y, 20.0, 0xffffff00);
	  //cout << "Centroid[" << i << "]: (" << x << ", " << y << ")" << endl;
	}
      }
    
      if(im != NULL){
	vx_object_t *vim = vxo_image_from_u32(im,
					      VXO_IMAGE_FLIPY,
					      VX_TEX_MIN_FILTER | VX_TEX_MAG_FILTER);
	//use pix coords to make a fix image
	vx_buffer_add_back (buf,vxo_chain (
					   vxo_mat_translate3 (-state.im_width/2., -state.im_height/2., 0.),
					   vim));
      
      
      
      }
      vx_buffer_swap(buf);
      usleep(1000000/fps);
      image_u32_destroy(im);
    }
    pthread_mutex_unlock(&state.data_mutex);
  }
}

static int mouse_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse){

  pthread_mutex_lock(&state.data_mutex);
  if((mouse->button_mask & VX_BUTTON1_MASK) && !(state.last_mouse_event.button_mask & VX_BUTTON1_MASK)){
    vx_ray3_t ray;
    vx_camera_pos_compute_ray (pos, mouse->x, mouse->y, &ray);
    double ground[3];
    vx_ray3_intersect_xy(&ray, 0, ground);
    printf ("Mouse clicked at coords: [%8.3f, %8.3f] Ground clicked at coords: [%6.3f, %6.3f]\n",
	    mouse->x, mouse->y, ground[0], ground[1]);
    state.click_point.x = ground[0];
    state.click_point.y = ground[1];
    if(state.has_tx){
      ground[0] = ((state.im_width/2) + ground[0]);
      ground[1] = ((state.im_height/2) + ground[1]);
      ground[2] = 1;
      double ground_in_arm[3];
      ground_in_arm[0] = matd_get(state.tx_mat, 0, 0)*ground[0] + matd_get(state.tx_mat, 0, 1)*ground[1] + matd_get(state.tx_mat, 0, 2)*ground[2];
      ground_in_arm[1] = matd_get(state.tx_mat, 1, 0)*ground[0] + matd_get(state.tx_mat, 1, 1)*ground[1] + matd_get(state.tx_mat, 1, 2)*ground[2];
      ground_in_arm[2] = matd_get(state.tx_mat, 2, 0)*ground[0] + matd_get(state.tx_mat, 2, 1)*ground[1] + matd_get(state.tx_mat, 2, 2)*ground[2];
      printf("Mouse click in arm coords: (%f, %f, %f)\n", ground_in_arm[0], ground_in_arm[1], ground_in_arm[2]);
      if(state.nine_squares_count < 9){
	fprintf(state.nine_squares,"%f %f\n",ground_in_arm[0],ground_in_arm[1]);
	++state.nine_squares_count;
      }
      else if(state.nine_squares_count == 9){
	fclose(state.nine_squares);
	++state.nine_squares_count;
      }
    }
  }
  state.last_mouse_event = *mouse;
  pthread_mutex_unlock(&state.data_mutex);
}

static int key_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key){
  pthread_mutex_lock(&state.data_mutex);
  if(!key->released){
    if(key->key_code == VX_KEY_SPACE){
      /*
       *
       * This uses click position. If causes problems, use nearest cyan centroid
       *
       */
      fprintf(state.printfile, "%f %f %f %f\n", state.im_width/2.0 + state.click_point.x, state.im_height/2.0 + state.click_point.y, state.arm_coords.x, state.arm_coords.y);
      printf("%f %f %f %f\n", state.im_width/2.0 + state.click_point.x, state.im_height/2.0 + state.click_point.y, state.arm_coords.x, state.arm_coords.y);
    }
    if(key->key_code == 'c'){
      cout << "Closing file. Don't try to write any more" << endl;
      fclose(state.printfile);
      //state.printfile = fopen("../calibration/arm_mappings.txt","r");
      ifstream f1("../calibration/arm_mappings.txt");
      double a_data[36];
      double b_data[6];
      //cout << "File opened again" << endl;
      for(int i=0; i<3; ++i){
	//state.printfile >> a_data[i*12 +0] >> a_data[i*12 + 1];
	//fscanf(state.printfile, "%f %f %f %f\n", &a_data[i*12 +0], &a_data[i*12 +1], &b_data[i*2 +0], &b_data[i*2 +1]);
	f1 >> a_data[i*12 +0] >> a_data[i*12 +1] >> b_data[i*2 +0] >> b_data[i*2 +1];
	cout << a_data[i*12 +0] << ' ' <<  a_data[i*12 +1] << ' ' << b_data[i*2 +0] << ' ' << b_data[i*2 +1] << endl;
	//cout << "File scanned: " << i << endl;
	a_data[i*12 +2] = 1;
	a_data[i*12 +3] = 0;
	a_data[i*12 +4] = 0;
	a_data[i*12 +5] = 0;
	a_data[i*12 +6] = 0;
	a_data[i*12 +7] = 0;
	a_data[i*12 +8] = 0;
	a_data[i*12 +9] = a_data[i*12 +0];
	a_data[i*12 +10] = a_data[i*12 +1];
	a_data[i*12 +11] = 1;
	//state.printfile >> b_data[i*2 +0] >> b_data[i*2+1];
      }
      gsl_matrix_view m = gsl_matrix_view_array(a_data, 6, 6);
      gsl_vector_view b = gsl_vector_view_array(b_data, 6);
      gsl_vector *x = gsl_vector_alloc(6);
      int s;
      gsl_permutation *p = gsl_permutation_alloc(6);
      gsl_linalg_LU_decomp(&m.matrix, p, &s);
      gsl_linalg_LU_solve(&m.matrix, p, &b.vector, x);
      printf("x = \n");
      FILE *fp = fopen("../calibration/transform_elements.txt","w");
      gsl_vector_fprintf(fp, x, "%g");
      gsl_vector_fprintf(stdout, x, "%g");
      state.has_tx = true;
      state.tx_mat = matd_create(3, 3);
      matd_put(state.tx_mat, 0, 0, gsl_vector_get(x, 0));
      matd_put(state.tx_mat, 0, 1, gsl_vector_get(x, 1));
      matd_put(state.tx_mat, 0, 2, gsl_vector_get(x, 2));
      matd_put(state.tx_mat, 1, 0, gsl_vector_get(x, 3));
      matd_put(state.tx_mat, 1, 1, gsl_vector_get(x, 4));
      matd_put(state.tx_mat, 1, 2, gsl_vector_get(x, 5));
      matd_put(state.tx_mat, 2, 0, 0);
      matd_put(state.tx_mat, 2, 1, 0);
      matd_put(state.tx_mat, 2, 2, 1.0);
      gsl_permutation_free(p);
      gsl_vector_free(x);
      //fclose(state.printfile);
      fclose(fp);
      
    }
  }
  pthread_mutex_unlock(&state.data_mutex);
  return 0;
}

static int touch_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_touch_event_t *mouse){
  return 0;
}

static void display_finished(vx_application_t *app, vx_display_t *disp){
  pthread_mutex_lock(&state.mutex);
  vx_layer_t *layer = NULL;
  zhash_remove(state.layers, &disp, NULL, &layer);
  vx_layer_destroy(layer);
  //image_u32_destroy (state.im);
  matd_destroy(state.tx_mat);
  pthread_mutex_unlock(&state.mutex);
}

static void display_started(vx_application_t *app, vx_display_t *disp){
  vx_layer_t *layer = vx_layer_create(state.vxworld);
  vx_layer_set_display(layer, disp);
  if(state.vxeh != NULL){
    vx_layer_add_event_handler(layer,state.vxeh);
  }
  pthread_mutex_lock(&state.mutex);
  zhash_put(state.layers, &disp, &layer, NULL, NULL);
  pthread_mutex_unlock(&state.mutex);
}

static void init_stuff(){
  //GUI init stuff
  state.vxworld = vx_world_create();
  printf("create world\n");
  state.vxeh = (vx_event_handler_t*) calloc(1,sizeof(*state.vxeh));
  state.vxeh->mouse_event = mouse_event;
  state.vxeh->touch_event = touch_event;
  state.vxeh->key_event = key_event;
  state.vxeh->dispatch_order = 100;
  state.vxeh->impl = &state;
  printf("create vxeh\n");
  state.vxapp.impl= &state;
  state.vxapp.display_started = display_started;
  state.vxapp.display_finished = display_finished;
  state.layers = zhash_create(sizeof(vx_display_t*), sizeof(vx_layer_t*), zhash_ptr_hash, zhash_ptr_equals);
  
  pthread_mutex_init (&state.mutex, NULL);
  pthread_mutex_init (&state.data_mutex,NULL);
  
  state.has_tx = false;
  state.gopt = getopt_create(); 
  FILE *fp = fopen("../calibration/mask_rect.txt","r");
  state.nine_squares = fopen("../calibration/nine_squares.txt","w");
  state.nine_squares_count = 0;
  fscanf(fp,"%f %f %f %f\n",&state.corner_coords[0].x,&state.corner_coords[1].x,&state.corner_coords[0].y,&state.corner_coords[1].y);
  printf("coord: %f %f %f %f\n",state.corner_coords[0].x,state.corner_coords[1].x,state.corner_coords[0].y,state.corner_coords[1].y);
  //red_hsv.read_hsv_from_file("../calibration/red_hsv_range.txt");
  //green_hsv.read_hsv_from_file("../calibration/green_hsv_range.txt");
  state.cyan_hsv.read_hsv_from_file("../calibration/cyan_hsv_range.txt");

  state.printfile = fopen("../calibration/arm_mappings.txt","w");
}

static void destroy_stuff(){
  fclose(state.printfile);
  vx_world_destroy(state.vxworld);
  if(zhash_size(state.layers) != 0){
    zhash_destroy(state.layers);
  }
  if(state.gopt){
    getopt_destroy(state.gopt);
  }
  lcm_destroy(state.lcm);
  pthread_mutex_destroy(&state.mutex);
}

int main(int argc, char *argv[]){
  
  init_stuff();

  getopt_add_bool (state.gopt, 'h', "help", 0, "Show this help screen");
  getopt_add_bool (state.gopt, 'i', "idle", 0, "Command all servos to idle");
  getopt_add_string (state.gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
  getopt_add_string (state.gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");
  getopt_add_string (state.gopt, 'f', "picurl", "", "Picture URL");


  if (!getopt_parse (state.gopt, argc, argv, 1) || getopt_get_bool (state.gopt, "help")) {
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

  state.lcm = lcm_create (NULL);
  state.command_channel = getopt_get_string (state.gopt, "command-channel");
  state.status_channel = getopt_get_string (state.gopt, "status-channel");

  //vx
  gdk_threads_init();
  gdk_threads_enter();
  gtk_init(&argc, &argv);

  state.status_thread = thread(status_loop);
  state.command_thread = thread(command_loop);
  state.animate_thread = thread(render_loop);

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



  state.status_thread.join();
  state.command_thread.join();
  state.animate_thread.join();

  destroy_stuff();
  return 0;
}
