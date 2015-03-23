#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <thread>
#include <lcm/lcm-cpp.hpp>
#include <vector>
#include <deque>
#include <iostream>
#include <string>
#include "math/point.hpp"

#include "lcmtypes/ttt_turn_t.h"

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
#include "calibration.hpp"
#include "board_state.hpp"
#include "gameboard.hpp"
#include "a2_inv_kin.hpp"
#include "arm_ai.hpp"
using namespace std;

typedef struct state state_t;
struct state
{
  getopt_t *gopt;
  lcm_t *lcm;

  const char *command_channel;
  const char *status_channel;
  const char *recv_chan;
  const char *send_chan;

  char *camera_url;
  char *pic_url;
  bool usePic;
  
  vx_application_t vxapp;
  vx_world_t *vxworld;
  zhash_t *layers;
  vx_event_handler_t *vxeh;
  vx_mouse_event_t last_mouse_event;
  vx_gtk_display_source_t *appwrap;

  eecs467::Point<float> corner_coords[2];
  eecs467::Point<float> click_point;

  image_source_t *isrc;
  int im_width;
  int im_height;

  calibration_t cal;
  inv_kinematics inv_kin;
  image_processor im_processor;
  BoardState board_state;
  gameboard game_board;
  arm_ai    ai;

  char color;
  bool our_turn;
  int green_turn_num;
  int red_turn_num;

  vector<int> continuous_green_centers;
  vector<int> continuous_red_centers;
  
  thread status_thread;
  thread command_thread;
  thread animate_thread;
  thread run_thread;
  thread send_thread;

  pthread_mutex_t mutex;
  pthread_mutex_t data_mutex;
  pthread_mutex_t arm_mutex;
};
state_t state;

static void status_handler(const lcm_recv_buf_t *rbuf,
			   const char *channel,
			   const dynamixel_status_list_t *msg,
			   void *user){
  //don't do anything

}

static void turn_handler(const lcm_recv_buf_t *rbuf,
			 const char *channel,
			 const ttt_turn_t *msg,
			 void *user){
  //don't do anything
  pthread_mutex_lock(&state.arm_mutex);
  //cout << "Received msg " << msg->turn << endl;
  if(state.color == 'G'){
    if(state.red_turn_num != msg->turn)
      state.our_turn = true;
    state.red_turn_num = msg->turn;
  }
  else if(state.color == 'R'){
    if(state.green_turn_num != msg->turn)
      state.our_turn = true;
    state.green_turn_num = msg->turn;
  }
  pthread_mutex_unlock(&state.arm_mutex);
}

void status_loop(){
  dynamixel_status_list_t_subscribe (state.lcm,
				     state.status_channel,
				     status_handler,
				     &state);
  cout << "status loop" << endl;
  
  ttt_turn_t_subscribe (state.lcm,
			state.recv_chan,
			turn_handler,
			&state);
  while(1){
    lcm_handle(state.lcm);
  }
}

void command_loop(){
  while(state.color != 'G' and state.color != 'R'){}
  //usleep(1000000);
  state.ai = arm_ai(state.color);
  state.inv_kin.go_home();
  if(state.color == 'G'){
    state.our_turn = false;
  }
  else
    state.our_turn = true;
  cout << "command_loop" << endl;
  
  while(!state.game_board.is_finished() && (state.game_board.is_win(state.ai.get_player()) == 0)){
    pthread_mutex_lock(&state.arm_mutex);
    int i = 0;
    while(!state.our_turn){
      pthread_mutex_unlock(&state.arm_mutex);
      if(!i)
	cout << "Waiting for turn" << endl;
      ++i;
      //state.inv_kin.wave();
      pthread_mutex_lock(&state.arm_mutex);
    }
    pthread_mutex_unlock(&state.arm_mutex);
    if(state.game_board.is_finished() || (state.game_board.is_win(state.ai.get_player()) != 0)){
      break;
    }
    gameboard new_board;
    do{
      cout << '\t' << "Doing turn num " << state.green_turn_num << endl;
      std::vector<int> red_center_list;
      std::vector<int> green_center_list;
      std::vector<int> cyan_center_list;
      image_u32_t* im;
      pthread_mutex_lock(&state.data_mutex);
      image_source_data_t *frmd = (image_source_data_t*) calloc(1, sizeof(image_source_data_t));
      int res = state.isrc->get_frame(state.isrc, frmd);
      if(res < 0){
	printf("get_frame fail\n");
      }
      else{
	im = image_convert_u32(frmd);
      }
      fflush(stdout);
      state.isrc->release_frame(state.isrc,frmd);

      if(state.corner_coords[0].x != -1 && 
	 state.corner_coords[0].y != -1 && 
	 state.corner_coords[1].x != -1 && 
	 state.corner_coords[1].y != -1)
	{  
	  state.im_processor.image_masking(im,
					   state.corner_coords[0].x,
					   state.corner_coords[1].x,
					   state.corner_coords[0].y,
					   state.corner_coords[1].y);
	  red_center_list = state.im_processor.blob_detection(im,
							      state.corner_coords[0].x,
							      state.corner_coords[1].x,
							      state.corner_coords[0].y,
							      state.corner_coords[1].y,
							      state.cal.get_red());
	  green_center_list = state.im_processor.blob_detection(im,
								state.corner_coords[0].x,
								state.corner_coords[1].x,
								state.corner_coords[0].y,
								state.corner_coords[1].y,
								state.cal.get_green()); 
	  cyan_center_list = state.im_processor.blob_detection(im, 
							       state.corner_coords[0].x,
							       state.corner_coords[1].x,
							       state.corner_coords[0].y,
							       state.corner_coords[1].y,
							       state.cal.get_cyan()); 
	} 
      max_min_hsv cyan = state.cal.get_cyan();
      cout << cyan.get_max_HSV().H << " " << cyan.get_max_HSV().S << " " << cyan.get_max_HSV().V << endl;
      image_u32_write_pnm(im, "picture_no_blobs");
      pthread_mutex_unlock(&state.data_mutex);
      if(cyan_center_list.empty())
	cout << "Cyan1 squares not detected. Recalibrate" << endl;
      if(green_center_list.empty())
	cout << "The curious case of the vanishing green1 balls" << endl;
      if(red_center_list.empty())
	cout << "The curious case of the vanishing red1 balls" << endl;

      state.game_board.update_entire_board(
					   state.board_state.determineStateofBoard(
										   green_center_list,red_center_list,
										   im->width,im->height,state.cal, state.color));

      state.game_board.print_board();

      int pos = state.ai.calc_move(state.game_board.get_board(state.ai.get_player()));
      
      eecs467::Point<double> free_ball;
      if(state.board_state.ballsLeft()){
	free_ball = state.board_state.nextFreeBall();
	std::cout << '\t' << free_ball.x << ' ' << free_ball.y << std::endl;
      }
      else{
	cout << "No free balls." << endl;
	continue;
      }
    
      cout << "Pick up @ " << free_ball.x << ' ' << free_ball.y << endl;
      state.inv_kin.pick_up(free_ball.x, free_ball.y);

      //state.inv_kin.place_08(pos);
      eecs467::Point<double> arm_pos = state.board_state.board_squares[pos];
      cout << "Place @ " << arm_pos.x << " " << arm_pos.y << endl;
      state.inv_kin.place(arm_pos.x, arm_pos.y);
      image_u32_destroy(im);
      
      frmd = (image_source_data_t*) calloc(1, sizeof(image_source_data_t));
      res = state.isrc->get_frame(state.isrc, frmd);
      if(res < 0){
	printf("get_frame fail\n");
      }
      else{
	im = image_convert_u32(frmd);
      }
      fflush(stdout);
      state.isrc->release_frame(state.isrc,frmd);

      if(state.corner_coords[0].x != -1 && 
	 state.corner_coords[0].y != -1 && 
	 state.corner_coords[1].x != -1 && 
	 state.corner_coords[1].y != -1)
	{  
	  state.im_processor.image_masking(im,
					   state.corner_coords[0].x,
					   state.corner_coords[1].x,
					   state.corner_coords[0].y,
					   state.corner_coords[1].y);
	  red_center_list = state.im_processor.blob_detection(im,
							      state.corner_coords[0].x,
							      state.corner_coords[1].x,
							      state.corner_coords[0].y,
							      state.corner_coords[1].y,
							      state.cal.get_red());
	  green_center_list = state.im_processor.blob_detection(im,
								state.corner_coords[0].x,
								state.corner_coords[1].x,
								state.corner_coords[0].y,
								state.corner_coords[1].y,
								state.cal.get_green()); 
	  cyan_center_list = state.im_processor.blob_detection(im, 
							       state.corner_coords[0].x,
							       state.corner_coords[1].x,
							       state.corner_coords[0].y,
							       state.corner_coords[1].y,
							       state.cal.get_cyan()); 
	} 
      if(cyan_center_list.empty())
	cout << "Cyan squares not detected. Recalibrate" << endl;
      if(green_center_list.empty())
	cout << "The curious case of the vanishing green balls" << endl;
      if(red_center_list.empty())
	cout << "The curious case of the vanishing red balls" << endl;

      new_board.update_entire_board(
				    state.board_state.determineStateofBoard(
									    green_center_list,red_center_list,
									    im->width,im->height,state.cal, state.color));

      new_board.print_board();

      if(new_board.is_finished() || (new_board.is_win(state.ai.get_player()) != 0)){
	int win = new_board.is_win(state.ai.get_player());
	if(win == 1){
	  cout << "We WIN!!" << endl;
	  for(int i=0; i<10; ++i){
	    state.inv_kin.wave();
	  }
	}
	else if(win == 0){
	  cout << "'tis a draw" << endl;
	}
	else{
	  cout << "We lost" << endl;
	}
	exit(0);
	//break;
      }

    } while(new_board == state.game_board);
      
    //}while(0);
    state.our_turn = false;
    //ttt_turn_t msg;
    //msg.utime = utime_now();
    pthread_mutex_lock(&state.data_mutex);
    if(state.color == 'G'){
      //msg.turn = state.green_turn_num;
      ++state.green_turn_num;
    }
    else{
      //msg.turn = state.red_turn_num;
      ++state.red_turn_num;
    }    
    pthread_mutex_unlock(&state.data_mutex);
    /*
    ttt_turn_t_publish(state.lcm,
		       state.send_chan,
		       &msg);*/
  }


}

static void msg_send_loop(){
  int hz = 20;
  while(1){
    ttt_turn_t msg;
    pthread_mutex_lock(&state.data_mutex);
    if(state.color == 'G')
      msg.turn = state.green_turn_num;
    else if(state.color == 'R')
      msg.turn = state.red_turn_num;
    pthread_mutex_unlock(&state.data_mutex);
    msg.utime = utime_now();
    //cout << 'R' << " sending turn num " << state.red_turn_num << endl;
    //cout << 'G' << " sending turn num " << state.green_turn_num << endl << endl << endl;
    ttt_turn_t_publish(state.lcm,
		       state.send_chan,
		       &msg);
    usleep(1000000/hz);
  }
}

static void render_loop()
{
  while(state.color != 'R' and state.color != 'G'){}
  cout << "Render loop" << endl;
  usleep(1000000);
  cout << "Starting Camera " << state.camera_url << endl;
  state.isrc = NULL;
  state.isrc = image_source_open(state.camera_url);
  if(state.isrc == NULL){
    printf("Error opening device\n");
  }
  else{
    state.isrc->start(state.isrc);
  }
  int fps = 60;
  while (1) {
    pthread_mutex_lock(&state.data_mutex);
    vx_buffer_t *buf = vx_world_get_buffer(state.vxworld,"image");
    image_u32_t *im; 
    std::vector<int> red_center_list;
    std::vector<int> green_center_list;
    std::vector<int> cyan_center_list;
    if(state.usePic){
      im = image_u32_create_from_pnm(state.pic_url); 
    }
    else if(state.isrc != NULL){
      image_source_data_t *frmd = (image_source_data_t*) calloc(1,sizeof(*frmd));
      int res = state.isrc->get_frame(state.isrc,frmd);
      if(res < 0){
	printf("get_frame fail\n");
      }
      else{
	im = image_convert_u32(frmd);
      }
      fflush(stdout);
      state.isrc->release_frame(state.isrc,frmd);
    }
    if(state.corner_coords[0].x != -1 && 
       state.corner_coords[0].y != -1 && 
       state.corner_coords[1].x != -1 && 
       state.corner_coords[1].y != -1){  
      state.im_processor.image_masking(im,
				       state.corner_coords[0].x,
				       state.corner_coords[1].x,
				       state.corner_coords[0].y,
				       state.corner_coords[1].y);
      state.continuous_red_centers = state.im_processor.blob_detection(im,
								       state.corner_coords[0].x,
								       state.corner_coords[1].x,
								       state.corner_coords[0].y,
								       state.corner_coords[1].y,
								       state.cal.get_red());
      state.continuous_green_centers = state.im_processor.blob_detection(im,
									 state.corner_coords[0].x,
									 state.corner_coords[1].x,
									 state.corner_coords[0].y,
									 state.corner_coords[1].y,
									 state.cal.get_green()); 
      cyan_center_list = state.im_processor.blob_detection(im, 
							   state.corner_coords[0].x,
							   state.corner_coords[1].x,
							   state.corner_coords[0].y,
							   state.corner_coords[1].y,
							   state.cal.get_cyan()); 
    } 
    if(!state.continuous_red_centers.empty()){
      for(int i=0;i<state.continuous_red_centers.size();++i){
	int y = (state.continuous_red_centers[i])/im->width;
	int x = (state.continuous_red_centers[i])%im->width;
	state.im_processor.draw_circle(im,x,y,10.0,0xff0000ff);
      }
    }
    if(!state.continuous_green_centers.empty()){
      for(int i=0;i<state.continuous_green_centers.size();++i){
	int y = (state.continuous_green_centers[i])/im->width;
	int x = state.continuous_green_centers[i]%im->width;
	state.im_processor.draw_circle(im,x,y,10.0,0xff00ff00);
      }
    }
    if(!cyan_center_list.empty()){
      for(int i=0; i < cyan_center_list.size(); ++i){
	int y = (cyan_center_list[i]) / im->width;
	int x = cyan_center_list[i] % im->width;
	state.im_processor.draw_circle(im, x, y, 20.0, 0xffffff00);
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
    pthread_mutex_unlock(&state.data_mutex);
    vx_buffer_swap(buf);
    usleep(1000000/fps);
  }
}

static int mouse_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_camera_pos_t *pos, vx_mouse_event_t *mouse){
  return 0;
}

static int key_event(vx_event_handler_t *vxeh, vx_layer_t *vl, vx_key_event_t *key){
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


  state.cal.read_tx_mat("../calibration/transform_elements.txt");

  state.cal.read_mask("../calibration/mask_rect.txt");
  vector<float> xys = state.cal.get_mask();
  state.corner_coords[0].x = xys[0];
  state.corner_coords[1].x = xys[1];
  state.corner_coords[0].y = xys[2];
  state.corner_coords[1].y = xys[3];

  state.cal.read_red_range("../calibration/red_hsv_range.txt");
  state.cal.read_green_range("../calibration/green_hsv_range.txt");
  state.cal.read_cyan_range("../calibration/cyan_hsv_range.txt");

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
  pthread_mutex_init (&state.arm_mutex,NULL);

  state.gopt = getopt_create(); 
}

static void destroy_stuff(){
  vx_world_destroy(state.vxworld);
  if(zhash_size(state.layers) != 0){
    zhash_destroy(state.layers);
  }
  if(state.gopt){
    getopt_destroy(state.gopt);
  }
  lcm_destroy(state.lcm);
  pthread_mutex_destroy(&state.mutex);
  pthread_mutex_destroy(&state.data_mutex);
  pthread_mutex_destroy(&state.arm_mutex);
}

int main(int argc, char *argv[]){
  state.lcm = lcm_create (NULL);
  init_stuff();
  state.color = 'q';
  while(state.color != 'G' && state.color != 'R'){
    cout << "Which color are you playing as?" << endl;
    cin >> state.color;
  }

  getopt_add_bool (state.gopt, 'h', "help", 0, "Show this help screen");
  getopt_add_bool (state.gopt, 'i', "idle", 0, "Command all servos to idle");
  getopt_add_string (state.gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
  getopt_add_string (state.gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");
  getopt_add_string (state.gopt, '\0', "green-channel", "GREEN_TURN", "LCM green channel");
  getopt_add_string (state.gopt, '\0', "red-channel", "RED_TURN", "LCM red channel");
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

  

  state.command_channel = getopt_get_string (state.gopt, "command-channel");
  state.status_channel = getopt_get_string (state.gopt, "status-channel");
  //state.recv_chan = getopt_get_string (state.gopt, "receive-channel");
  //state.send_chan = getopt_get_string (state.gopt, "send-channel");
  if(state.color == 'G'){
    state.send_chan = getopt_get_string(state.gopt, "green-channel");
    state.recv_chan = getopt_get_string(state.gopt, "red-channel");
  }
  if(state.color == 'R'){
    state.send_chan = getopt_get_string(state.gopt, "red-channel");
    state.recv_chan = getopt_get_string(state.gopt, "green-channel");
  }
  cout << "post gopt " << state.send_chan << endl;
  //vx
  gdk_threads_init();
  gdk_threads_enter();
  gtk_init(&argc, &argv);

  state.inv_kin.set_lcm(state.lcm);
  state.inv_kin.set_com_channel(state.command_channel);
  state.inv_kin.go_home();

  state.status_thread = thread(status_loop);
  state.command_thread = thread(command_loop);
  state.animate_thread = thread(render_loop);
  state.send_thread = thread(msg_send_loop);

  state.appwrap = vx_gtk_display_source_create(&state.vxapp);
  GtkWidget * window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  GtkWidget * canvas = vx_gtk_display_source_get_widget(state.appwrap);
  gtk_window_set_default_size (GTK_WINDOW (window), 640, 480);
  gtk_container_add(GTK_CONTAINER(window), canvas);
  gtk_widget_show (window);
  cout << "showing stuff" << endl;
  gtk_widget_show (canvas); // XXX Show all causes errors!
  g_signal_connect_swapped(G_OBJECT(window), "destroy", G_CALLBACK(gtk_main_quit), NULL);
  gtk_main(); // Blocks as long as GTK window is open

  gdk_threads_leave();
  vx_gtk_display_source_destroy(state.appwrap);



  state.status_thread.join();
  state.command_thread.join();
  state.animate_thread.join();
  state.send_thread.join();

  destroy_stuff();
  return 0;
}
