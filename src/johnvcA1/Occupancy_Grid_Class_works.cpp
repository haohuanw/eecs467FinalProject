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

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"
#include "lcm/lcm-cpp.hpp"
#include "lcmtypes/maebot_pose_t.hpp"

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "math/point.hpp"
#include "eecs467_util.h"    // This is where a lot of the internals live
#include "math/gsl_util_rand.h"
#include "math/angle_functions.hpp"
#include <algorithm>
#include <queue>
#include <stack>

using namespace std;

//for switch statements 
int stopped = 2;
int turning = 2;
int ray_tracing = 0;
bool method_2 = false;

lcm::LCM lcm_inst;
pthread_mutex_t dog;

bool has_read = false;
bool initial_tick_read = true;

/*state structure holds the data for pose and
  lidar used by the compute the occupancy grid
  
  **there are 2 state structs used in this file,
  one for reading the data in, and one for 
  computing with, this removes the potential for 
  overwriting the data we're trying to compute 
  as we're reading in new data 
*/
struct state{
  long timestamp_0;
  long timestamp_1;
  float x_rob_0, x_rob_1;
  float y_rob_0, y_rob_1;
  float x_err, y_err, theta_err;
  float theta_rob_0, theta_rob_1;
  float vx, vy, w;
  int num_ranges;
  vector<float> ranges;
  vector<float> thetas;
  vector<long> times;
  vector<float> intensities;

  state(){
    timestamp_0 = timestamp_1 = 0;
    x_rob_0 = y_rob_0 = theta_rob_0 = 0;
   x_rob_1 = y_rob_1 = theta_rob_1  = 0;
    x_err = y_err = theta_err = 0;
    vx = vy = w = 0;
  }
};

//smallest unit of the particle filter
struct particle{
  float x;
  float y;
  float theta;
  float prob;
  long utime;
  particle(){
    x = y = theta = prob = 0;
    utime = 0;
  }
};

struct sort_funct{
  bool operator() (const particle i, particle j){
    return(i.prob < j.prob);
  }
};

//class for handling occupancy grid shit
class occ_grid{
public:
  
  stack<eecs467::Point<int>> path;
  queue<eecs467::Point<int>> path_find;

  bool new_data;
  state comp;
  state shared;
  eecs467::OccupancyGrid OG;
  eecs467::OccupancyGrid VG;
  gsl_rng *random_number;

  int des_x;
  int des_y;

  vector<particle> list_A;
  vector<particle> list_B;
  vector<particle> list_A_m;
  //vector
  int Rticks_0;
  int Lticks_0;
  float ticker;
  int highest_prob_0;
  int count;

  occ_grid() : OG(5.0f, 5.0f, 0.05) , VG(5.0f, 5.0f, 0.05) {
    new_data = false;
    OG.reset();
    VG.reset();

    des_x = des_y = 0;

    list_A.resize(2000);
    list_B.resize(2000);
    random_number = gslu_rand_rng_alloc ();

    for(int i = 0; i < int(list_A.size()); i++)
    {
	list_A[i].x = list_A[i].y = list_A[i].theta = 0;
	list_B[i].x = list_B[i].y = list_B[i].theta = 0;
	list_A[i].prob = list_B[i].prob = 0.01;
    }
    ticker = 0.0001;
    highest_prob_0 = 0;
    count = 0;
  }

  //===================== LCM HANDLERS =====================================================================================

  void maebot_odometry_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_motor_feedback_t* msg)
  {
    //set the initial tick count for left and right ticks
    if(initial_tick_read)
    {
      Rticks_0 = msg->encoder_right_ticks;
      Lticks_0 = msg->encoder_left_ticks;
      initial_tick_read = false;
      count ++;
    }
    //if we have new lidar data
    if(has_read == true){
      if( ((Lticks_0 > msg->encoder_left_ticks && Rticks_0 <= msg->encoder_right_ticks) ||
	  (Rticks_0 > msg->encoder_right_ticks && Lticks_0 <= msg->encoder_left_ticks)) && count > 2){
	//cout << "turning" << endl;
	switch(turning){
	case 1:
	  //localize and map in a turn
	  localization(msg->encoder_right_ticks, msg->encoder_left_ticks, msg->utime);
	  new_data = true;
	  break;
	case 2:
	  //localize in a turn
	  localization(msg->encoder_right_ticks, msg->encoder_left_ticks, msg->utime);
	  break;
	case 3:
	  //map in a turn, need to update localization time
	  for(int i = 0; i < int(list_A.size()); i++){
	    list_A[i].utime = msg->utime;
	  }
	  new_data = true;
	default:
	  new_data = false;
	  for(int i = 0; i < int(list_A.size()); i++){
	    list_A[i].utime = msg->utime;
	  }
	}
      }
      else if( Lticks_0 == msg->encoder_left_ticks && Rticks_0 == msg->encoder_right_ticks && count > 2 ){
	//cout << "stopped" << endl;
	switch(stopped){
	case 1:
	  //localize and map when stopped
	  localization(msg->encoder_right_ticks, msg->encoder_left_ticks, msg->utime);
	  new_data = true;
	  break;
	case 2:
	  //localize in a when stopped
	  localization(msg->encoder_right_ticks, msg->encoder_left_ticks, msg->utime);
	  break;
	case 3:
	  //map in a when stopped, need to update localization time
	  for(int i = 0; i < int(list_A.size()); i++){
	    list_A[i].utime = msg->utime;
	  }
	  new_data = true;
	default:
	  new_data = false;
	  for(int i = 0; i < int(list_A.size()); i++){
	    list_A[i].utime = msg->utime;
	  }
	}
      }
      else{
	//cout << "normal" << endl;
	localization(msg->encoder_right_ticks, msg->encoder_left_ticks, msg->utime);
	new_data = true;
	count++;
      }
      
      /*now all the publishing done in the pose handler rather than in the 
      localization function so if we choose not to run localization somewhere
      we still will send the old value to the mapping function*/

    //publish our position over LCM
    maebot_pose_t msg_ptr;
    msg_ptr.x = list_A[highest_prob_0].x;
    msg_ptr.y = list_A[highest_prob_0].y;
    msg_ptr.theta = list_A[highest_prob_0].theta;
    msg_ptr.utime = list_A[highest_prob_0].utime;
    // cout << "new best x: " << msg_ptr.x << " y: " << msg_ptr.y << endl;
    lcm_inst.publish("MAEBOT_TASK2", &msg_ptr);

    //update the shared struct data
    pthread_mutex_lock(&dog);
    shared.theta_rob_0 = shared.theta_rob_1;
    shared.theta_rob_1 = list_A[highest_prob_0].theta;
    shared.timestamp_0 = shared.timestamp_1;
    shared.timestamp_1 = list_A[highest_prob_0].utime;
    shared.x_rob_0 = shared.x_rob_1;
    shared.y_rob_0 = shared.y_rob_1;
    shared.x_rob_1 = list_A[highest_prob_0].x;
    shared.y_rob_1 = list_A[highest_prob_0].y;
    pthread_mutex_unlock(&dog);

    if( path.empty() ){
      path_finding(shared.x_rob_1, shared.y_rob_1, shared.theta_rob_1);
    }
    

    }

  }

  /* NO LONGER NEEDED BECAUSE WE HAVE SLAM NOW
  void maebot_pose_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_pose_t* msg)
  {
    pthread_mutex_lock(&dog);
    shared.theta_rob_0 = shared.theta_rob_1;
    shared.theta_rob_1 = msg->theta;
    shared.timestamp_0 = shared.timestamp_1;
    shared.timestamp_1 = msg->utime;
    shared.x_rob_0 = shared.x_rob_1;
    shared.y_rob_0 = shared.y_rob_1;
    shared.x_rob_1 = msg->x;
    shared.y_rob_1 = msg->y;
    pthread_mutex_unlock(&dog);

    new_data = true;

    }*/

  void laser_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_laser_scan_t* msg)
  {
    pthread_mutex_lock(&dog);
    shared.num_ranges = msg->num_ranges;
    shared.ranges = msg->ranges;
    shared.thetas = msg->thetas;
    shared.times = msg->times;
    shared.intensities = msg->intensities;
    pthread_mutex_unlock(&dog);
    has_read = true;
  }

  //====================== POSE AND OCCUPANCY GRID FUCNTIONS =============================================================


  //=========localization==============
  void localization(int encoder_right_ticks, int encoder_left_ticks, long mtime){
    has_read = false; 
    int highest_prob = 0;
    float largest = -FLT_MAX;
    float norm_fact = 0;
    float cnt = 0;
    float cum_prob = list_A[0].prob;
    float inc = float(1.0/float(list_A.size()));
    int index = 0;
    
    //iterate through every element in the particle filter
    for(int i = 0; i < int(list_A.size()); i++)
      {
	particle temp;
	
	while(cnt >= cum_prob){
	  ++index;
	  if(index >= int(list_A.size())){
	    index = int(list_A.size())-1;
	    break;
	  }
	  cum_prob += list_A[index].prob;
	  
	}
	//cout << "index: " << index << endl;
	cnt = cnt + inc;

	assert(list_A[index].utime != mtime);
	//find the position and probability of the new particle, store it in temp
	temp = UpdateFromOdometry(list_A[index].x, list_A[index].y, list_A[index].theta, 
				  list_A[index].utime, encoder_right_ticks, encoder_left_ticks, mtime);
	
	//index i of the next particle filter is temp
	//if the probability of the position in temp is the largest so far,
	//save the highest probability
	list_B[i] = temp;
	if(temp.prob > largest)
	  {
	    highest_prob = i;
	    largest = temp.prob;
	  }
	
	
      }
    //set old ticks data
    Rticks_0 = encoder_right_ticks;
    Lticks_0 = encoder_left_ticks;
      
      
    //subtract every probability by the largest, find e to that prob, then add to normalizing factor
    for(int i = 0; i < int(list_B.size()); i++){
      list_B[i].prob -= largest;
      list_B[i].prob = exp(list_B[i].prob);
      norm_fact += list_B[i].prob;	
    }

    //divide every probability by the normalizing factor to get it to a number between 0 and 1
    for(int i = 0; i < int(list_B.size()); i++){
	list_B[i].prob = list_B[i].prob/norm_fact;
	//cout << "i: " << i << " x: "<<list_B[i].x<<" y "<<list_B[i].y<<" prob: " << list_B[i].prob << endl;
    }

    //move the new particle filter into the old particle filter
    list_A = list_B;
    /*if(method_2){
      std::sort(list_A.begin(), list_A.end(), sort_funct());
      highest_prob_0 = int(list_A.size())-1; 
    }
    else*/
      highest_prob_0 = highest_prob;
  }

   particle UpdateFromOdometry(float x, float y, float theta, long pose_time, int Rticks, int Lticks, long msg_time)
  {
     particle temp;
     float diff_left = (Lticks - Lticks_0);
     float diff_right = (Rticks - Rticks_0);
     diff_left = (diff_left/480)*M_PI*0.032;
     diff_right = (diff_right/480)*M_PI*0.032;

    // cout << "old tick_l: " << Lticks_0 << " tick_r " << Rticks_0 << endl;
    // cout << "new tick_l: " << Lticks << " tick_r " << Rticks << endl;	

     float turn = (diff_right - diff_left)/0.08;

     float distance = (diff_left + diff_right)/2;

     turn = gslu_rand_gaussian (random_number, turn, sqrt(0.04*abs(turn)));
     distance = gslu_rand_gaussian (random_number, distance, sqrt(0.003*abs(distance)));

     float x_change = distance*cos(turn/2 + theta);
     float y_change = distance*sin(turn/2 + theta);

     float theta_change = turn;
	
    // cout << "dx: " << x_change << " dy: " << y_change << " dtheta: " << theta_change << endl; 

     temp.utime = msg_time;

     temp.x = x + static_cast<float>(x_change);
     temp.y = y + static_cast<float>(y_change);

     temp.theta = eecs467::wrap_to_pi(theta + theta_change);
     temp.prob = float(CalculateProb(temp.x, temp.y, temp.theta, temp.utime, x, y, theta, pose_time)); 
     return temp;
  }

  int CalculateProb(float x_1, float y_1, float theta_1, long time_1, float x_0, float y_0, float theta_0, long time_0)
  {
    // Calculate the probability for this instance of particle
    //take the old most probable pose, the new pose in this particle, and find the velocities from that
    //then run the laser scan, a laser scan is correct if the laser stops at an occupied block, and passes through a free one
    eecs467::Point<double> world_coord;
    eecs467::Point<int> grid_coord;
    eecs467::Point<int> beam_end;
    float x_err, y_err, theta_err;
    float x_beam, y_beam, theta_w;
	assert(time_1 != time_0);
    float vx = (x_1 - x_0)/(time_1-time_0);
    float vy = (y_1 - y_0)/(time_1-time_0);
    float w = (eecs467::wrap_to_pi(theta_1 - theta_0))/(time_1 - time_0);
    //int odds;
    int count = 0;
    //bool ok = true;

    for(int i = 0; i < shared.num_ranges; i++){

      //find error due to motion
      x_err = vx*(shared.times[i] - time_0)/1000000;
      y_err = vy*(shared.times[i] - time_0)/1000000;
      theta_err = w*(shared.times[i]-time_0)/(1000000);

      theta_w = (theta_0 + theta_err) - shared.thetas[i];
      x_beam = shared.ranges[i]*cos(theta_w) + (x_0 + x_err);
      y_beam = shared.ranges[i]*sin(theta_w) + (y_0 + y_err);
      world_coord.x = double( x_beam );
      world_coord.y = double( y_beam );
      beam_end = global_position_to_grid_cell( world_coord, OG);

      world_coord.x = double( x_0 + x_err );
      world_coord.y = double( y_0 + y_err );
      grid_coord = global_position_to_grid_cell( world_coord, OG);
      
      switch( ray_tracing ){
	//using the end of the beam
      case 0:
	if(OG.isCellInGrid(beam_end.x, beam_end.y)){
	  if(OG.logOdds(beam_end.x, beam_end.y) > 0 )
	    count -= 2;
	  else if(OG.logOdds(beam_end.x, beam_end.y) < 0 )
	    count -= 4;
	  else
	    count -= 6;
	}
	break;
	//using ray tracing at 3 points
      case 1:
	float incr = 0.05;
	//int cntr = 0;
	while( (grid_coord.x < beam_end.x) && (grid_coord.y < beam_end.y) ){
       
	  if(OG.logOdds(grid_coord.x, grid_coord.y) > 0){
	    count -= 6;
	  }
	  else if(OG.logOdds(grid_coord.x, grid_coord.y) == 0)
	    count -=10;
	
	  //move another 5cm up the beam line
	  world_coord.x += incr*cos(theta_w);
	  world_coord.y += incr*sin(theta_w);
	  //get grid positions from global positions
	  grid_coord = global_position_to_grid_cell( world_coord, OG);
	}
	if(OG.isCellInGrid(beam_end.x, beam_end.y)){
	  if(OG.logOdds(beam_end.x, beam_end.y) > 0 )
	    count -= 1;
	  else if(OG.logOdds(beam_end.x, beam_end.y) < 0 )
	    count -= 8;
	  else
	    count -= 12;
	}

	break;
      }

    }
    //cout << count << endl;
    return count; // <- return the probability
  }

  //==== Occupancy Grid =======================

  bool has_new_data(){
    bool ret;
    pthread_mutex_lock(&dog);
    ret = new_data;
    pthread_mutex_unlock(&dog);
    return ret;
  }

  //copies shared state to the computational state
  void share2comp(){
    pthread_mutex_lock(&dog);
    comp = shared;
    new_data = false;
    pthread_mutex_unlock(&dog);
  }

  //finds the error in the robot's position from motion
  void motion_error(long time ){
    comp.vx = (comp.x_rob_1 - comp.x_rob_0)/(comp.timestamp_1 - comp.timestamp_0);
    comp.vy = (comp.y_rob_1 - comp.y_rob_0)/(comp.timestamp_1 - comp.timestamp_0);
    comp.w = eecs467::wrap_to_pi(comp.theta_rob_1 - comp.theta_rob_0)/(comp.timestamp_1 - comp.timestamp_0);

    comp.x_err = comp.vx*(time - comp.timestamp_0);
    comp.y_err = comp.vy*(time - comp.timestamp_0);
    comp.theta_err = comp.w*(time - comp.timestamp_0);
 
  }

  //computation function that updates the occupancy grid
  void compute(){
    share2comp();

    float theta_w;
    float x_beam;
    float y_beam;
    eecs467::Point<double> world_coord;
    eecs467::Point<int> grid_coord;
    eecs467::Point<int> beam_end;
    int8_t odds;

    //iterate through all beams
    for(int i = 0; i < comp.num_ranges; i++){
      //find the x,y coordinates of the endpoint of the lidar beam

      motion_error( comp.times[i] );

      theta_w  = eecs467::wrap_to_pi(eecs467::wrap_to_pi(comp.theta_rob_0 + comp.theta_err) - comp.thetas[i]);
      x_beam = comp.ranges[i]*cos(theta_w) + (comp.x_rob_0 + comp.x_err);
      y_beam = comp.ranges[i]*sin(theta_w) + (comp.y_rob_0 + comp.y_err);
      world_coord.x = double(x_beam); 
      world_coord.y = double(y_beam);
      beam_end = global_position_to_grid_cell( world_coord, OG);

      world_coord.x = double( comp.x_rob_0 + comp.x_err );
      world_coord.y = double( comp.y_rob_0 + comp.y_err );
      //get grid positions from global positions
      grid_coord = global_position_to_grid_cell( world_coord, OG);

      //decrement all cells passed by lidar beam
      //float len = 0;
      while( ( theta_w >= 0 && theta_w < M_PI/2 && grid_coord.x < beam_end.x && grid_coord.y < beam_end.y) ||
	     ( theta_w >= M_PI/2 && theta_w < M_PI && grid_coord.x > beam_end.x && grid_coord.y < beam_end.y) ||
	     ( theta_w >= -M_PI && theta_w < -M_PI/2 && grid_coord.x > beam_end.x && grid_coord.y > beam_end.y) ||
	     ( theta_w >= -M_PI/2 && theta_w < 0 && grid_coord.x < beam_end.x && grid_coord.y > beam_end.y ) ){ 
       
	if( OG.isCellInGrid(grid_coord.x, grid_coord.y) && !((grid_coord.x == beam_end.x) && (grid_coord.y == beam_end.y)) ){
	  //decrement logodds for a cell that's been passed through
	  odds = OG.logOdds(grid_coord.x, grid_coord.y);
	  if(odds > -126){
	    if(odds > -124 && odds < -30)
	      odds-= 4;
	    else
	      odds-=2;
	  }
	  OG.setLogOdds(grid_coord.x, grid_coord.y, odds);
	}
	else
	  break;
	
	//move another 5cm up the beam line
	world_coord.x += 0.05*cos(theta_w);
	world_coord.y += 0.05*sin(theta_w);
	//get grid positions from global positions
	grid_coord = global_position_to_grid_cell( world_coord, OG);
      }
      //incriment the logodds of the cell the beam ends at
      odds = OG.logOdds(beam_end.x, beam_end.y);
      if(odds < 122){
	if(odds < 107 && odds > 70)
	   odds+=20;
	else
	  odds+=5;
      }
      OG.setLogOdds(beam_end.x, beam_end.y, odds);

    }
    //publish the new occupancy grid and particle filter
    maebot_occupancy_grid_t msg = OG.toLCM();
    maebot_occupancy_grid_t *msg_ptr = &msg;
    lcm_inst.publish("MAEBOT_OCCUPANCY_GRID", msg_ptr);

  }

  //===================================================== PATH PLANNING ========================================

   bool check_cell(int x, int y)
  {
	if(!OG.isCellInGrid(x, y)) return false;
	if(VG.logOdds(x, y) != 0 || OG.logOdds(x,y) > 20) return false;
	return true;
  }

  eecs467::Point<int> find_Gray(int x, int y, int dir)
  {
      eecs467::Point<int> output;
      output.x = -1;
      output.y = -1;
      if(check_cell(x, y))
      {
          eecs467::Point<int> temp;
          temp.x = x;
          temp.y = y;
          VG.setLogOdds(temp.x, temp.y, dir);
	  pthread_mutex_lock(&dog);
          path_find.push(temp);
	  pthread_mutex_unlock(&dog);
	  if(OG.logOdds(temp.x, temp.y) > -20 && OG.logOdds(temp.x, temp.y) < 20)
          {
	       output.x = temp.x;
	       output.y = temp.y;
          }
 
       }
       return output;
  }

  void path_finding(float bot_x_pos, float bot_y_pos, float bot_theta)
  {
      VG.reset();
      while(!path_find.empty())
      {
	 path_find.pop();
      }
      eecs467::Point<double> world_coord;
      eecs467::Point<int> coord, des_coord, init_coord;

      world_coord.x = double(bot_x_pos); 
      world_coord.y = double(bot_y_pos);
      des_coord.x = -1;
      des_coord.y = -1;
      coord = global_position_to_grid_cell( world_coord, OG);
      init_coord = coord;
      VG.setLogOdds(coord.x, coord.y, 5);


      // Placing into queue //
      while(des_coord.x == -1 && des_coord.y == -1)
      {
          des_coord = find_Gray(coord.x - 1, coord.y - 1, 1);
          if(des_coord.x != -1 && des_coord.y != -1) break;

          des_coord = find_Gray(coord.x, coord.y - 1, 2);
          if(des_coord.x != -1 && des_coord.y != -1) break;

          des_coord = find_Gray(coord.x + 1, coord.y - 1, 3);
          if(des_coord.x != -1 && des_coord.y != -1) break;

          des_coord = find_Gray(coord.x - 1, coord.y, 4);
          if(des_coord.x != -1 && des_coord.y != -1) break;

          des_coord = find_Gray(coord.x + 1, coord.y, 6);
          if(des_coord.x != -1 && des_coord.y != -1) break;

          des_coord = find_Gray(coord.x - 1, coord.y + 1, 7);
          if(des_coord.x != -1 && des_coord.y != -1) break;

          des_coord = find_Gray(coord.x, coord.y + 1, 8);
          if(des_coord.x != -1 && des_coord.y != -1) break;

          des_coord = find_Gray(coord.x + 1, coord.y + 1, 9);
          if(des_coord.x != -1 && des_coord.y != -1) break;

	  pthread_mutex_lock(&dog);
	  coord = path_find.front();
	  path_find.pop();
	  pthread_mutex_unlock(&dog); 
     }

      // Find path

      coord = des_coord;
      while(VG.logOdds(coord.x, coord.y) != 5)
      {
 	  pthread_mutex_lock(&dog);
	  path.push(coord);
     	  pthread_mutex_unlock(&dog);
	  if(VG.logOdds(coord.x, coord.y) == 1)
	  {
	      coord.x = coord.x + 1;
 	      coord.y = coord.y + 1;
	  }
     	  else if(VG.logOdds(coord.x, coord.y) == 2)
	  {
	      coord.x = coord.x;
 	      coord.y = coord.y + 1;
	  }   
     	  else if(VG.logOdds(coord.x, coord.y) == 3)
	  {
	      coord.x = coord.x - 1;
 	      coord.y = coord.y + 1;
	  }   
     	  else if(VG.logOdds(coord.x, coord.y) == 4)
	  {
	      coord.x = coord.x + 1;
 	      coord.y = coord.y;
	  }   
     	  else if(VG.logOdds(coord.x, coord.y) == 6)
	  {
	      coord.x = coord.x - 1;
 	      coord.y = coord.y;
	  }   
     	  else if(VG.logOdds(coord.x, coord.y) == 7)
	  {
	      coord.x = coord.x + 1;
 	      coord.y = coord.y - 1;
	  }   
     	  else if(VG.logOdds(coord.x, coord.y) == 8)
	  {
	      coord.x = coord.x;
 	      coord.y = coord.y - 1;
	  }   
     	  else if(VG.logOdds(coord.x, coord.y) == 9)
	  {
	      coord.x = coord.x - 1;
 	      coord.y = coord.y - 1;
	  }   
      }
  }

  bool reach_location()
  {
     eecs467::Point<int> world_coord;
     eecs467::Point<int> coord;
     world_coord.x = double(shared.x_rob_1); 
     world_coord.y = double(shared.y_rob_1);
     coord = global_position_to_grid_cell( world_coord, OG);
     if(coord.x == des_x && coord.y == des_y && !path.empty())
     {
        pthread_mutex_lock(&dog);
	eecs467::Point<int> coord2 = path.top();
	path.pop();
	des_x = coord2.x;
	des_y = coord2.y;
        pthread_mutex_unlock(&dog);
	return true;
     }
     return false;
  }

  void do_motion()
  {
     cout << "in motion" << endl;
     if(!reach_location())
     {
     	eecs467::Point<int> world_coord;
     	world_coord.x = double(shared.x_rob_1); 
     	world_coord.y = double(shared.y_rob_1);

     	eecs467::Point<int> coord = global_position_to_grid_cell( world_coord, OG);

   	maebot_motor_command_t msg_ptr;
	float angle = 0;
     	if(des_x - coord.x == 1 && des_y - coord.y == 1)
     	{
	   // turn toward 9 direction
	   // find angle thats needed to be turned
	    angle = eecs467::wrap_to_pi(M_PI/4 - shared.theta_rob_1); 	 
     	}
     	if(des_x - coord.x == 0 && des_y - coord.y == 1)
     	{
     	    //turn toward 8 direction
	    angle = eecs467::wrap_to_pi(M_PI/2 - shared.theta_rob_1);   
     	}
     	if(des_x - coord.x == 1 && des_y - coord.y == 0)
     	{
     	    //turn toward 6 direction
	    angle = eecs467::wrap_to_pi(0 - shared.theta_rob_1);
     	}
     	if(des_x - coord.x == -1 && des_y - coord.y == -1)
    	{
            //turn toward 1 direction
	    angle = eecs467::wrap_to_pi(-3*M_PI/4 - shared.theta_rob_1);
     	}
     	if(des_x - coord.x == -1 && des_y - coord.y == 0)
     	{
     	    //turn toward 4 direction
	    angle = eecs467::wrap_to_pi(M_PI - shared.theta_rob_1);
     	}
     	if(des_x - coord.x == 0 && des_y - coord.y == -1)
     	{
     	    //turn toward 2 direction
	    angle = eecs467::wrap_to_pi(-M_PI/2 - shared.theta_rob_1);
     	}
     	if(des_x - coord.x == 1 && des_y - coord.y == -1)
     	{
     	    //turn toward 3 direction
	    angle = eecs467::wrap_to_pi(-M_PI/4 - shared.theta_rob_1);

     	}
     	if(des_x - coord.x == -1 && des_y - coord.y == 1)
     	{
     	    //turn towardshared.x_rob_1 7 direction
	    angle = eecs467::wrap_to_pi(3*M_PI/4 - shared.theta_rob_1);
     	}

	if(angle > 0.05)
	{
	    msg_ptr.motor_left_speed = -0.5;
	    msg_ptr.motor_right_speed = 0.5;
        }
	else if(angle < -0.05)
	{
	    msg_ptr.motor_left_speed = 0.5;
	    msg_ptr.motor_right_speed = -0.5;
	}
	else
	{
	    msg_ptr.motor_left_speed = 1;
	    msg_ptr.motor_right_speed = 1;
	}
	lcm_inst.publish("Drive_motor", &msg_ptr);

     }  
  }

};


//LCM COMMUNICATION THREAD
void *lcm_comm(void *input){

  while (true){
      lcm_inst.handle();
    }

    return NULL;
}

int main(int argc, char *argv[]){
  has_read = false;
  eecs467_init (argc, argv);

  occ_grid O;


  /*PSUEDOCODE FOR OCC GRID SHITS
    -initialize occupancy grid data structure using pre-written class declaration
      -define size in meters of grid
      
    -origin pose of the robot will be 0 degrees, at 0,0

    ****NOTE**** need to compensate for robot's motion, but start by assuming a stationary bot

    -when we recieve a lidar scan message
      -iterate through the lidar scan data
        -find the global position the lidar lands on
	  -subtract lidar theta from robot's theta to get global grid angle
          -find x,y components of lidar range for respective theta, add to robot's x,y
	  -from the robot's x,y and the global angle of the lidar beam, incriment up that angle by 5cm until 
           we reach the beam's coordinates, decrementing the log-odds of all cells we pass through as we go,
           finally incrimenting the log-odds of the final cell
          -repeat until we reach the end of the lidar vectors
           
	   
      -once done updating the map, save as image,  publish it to LCM for the GUI program to draw
        -how to mape to an image
        -how do we publish messages over LCM in cpp format
	-how to we draw it (handle this in GUI function)
  */
    
  //lcm_inst.subscribe("MAEBOT_POSE", &occ_grid::maebot_pose_handler, &O);
  lcm_inst.subscribe("MAEBOT_LASER_SCAN", &occ_grid::laser_handler, &O);
  lcm_inst.subscribe("MAEBOT_MOTOR_FEEDBACK", &occ_grid::maebot_odometry_handler, &O); 

  //initiate LCM thread
  //pthread_t pose_comm;
  //pthread_create(&pose_comm, NULL, lcm_comm , NULL);
  pthread_t lidar_comm;
  pthread_create(&lidar_comm, NULL, lcm_comm, NULL);

  while(true){

    if( O.has_new_data() ){
      O.compute();
      O.do_motion();
    }
    
  }

  return 0;
 }

