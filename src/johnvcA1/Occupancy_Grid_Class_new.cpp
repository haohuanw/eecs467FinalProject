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
pthread_mutex_t print;

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

  state(){
    timestamp_0 = timestamp_1 = 0;
    x_rob_0 = y_rob_0 = theta_rob_0 = 0;
   x_rob_1 = y_rob_1 = theta_rob_1  = 0;
    x_err = y_err = theta_err = 0;
    vx = vy = w = 0;
  }
};

struct data{

  //lidar data
  int num_ranges;
  vector<float> ranges;
  vector<float> thetas;
  vector<long> times;
  vector<float> intensities;

  //odometry data
  int Lticks_1, Lticks_0;
  int Rticks_1, Rticks_0;
  int Otime_1, Otime_0;

  data(){
    Lticks_1 = Lticks_0 = Rticks_1 = Rticks_0 = Otime_1 = Otime_0 = 0;
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

  //control booleans
  bool new_lidar;
  bool can_localize;
  bool can_update_og;
  bool init_odo;

  data comp;
  data shared;
  state r_state;
  state r_state_r;
  eecs467::OccupancyGrid OG;
  gsl_rng *random_number;

  vector<particle> list_A;
  vector<particle> list_B;
 
  int highest_prob_0;
  //int count;

  occ_grid() : OG(5.0f, 5.0f, 0.05) {
    new_lidar = can_localize = can_update_og = false;
    init_odo = true;
    OG.reset();

    list_A.resize(1000);
    list_B.resize(1000);
    random_number = gslu_rand_rng_alloc ();

    for(int i = 0; i < int(list_A.size()); i++)
    {
	list_A[i].x = list_A[i].y = list_A[i].theta = 0;
	list_B[i].x = list_B[i].y = list_B[i].theta = 0;
	list_A[i].prob = list_B[i].prob = 0.01;
    }
  
    highest_prob_0 = 0;
    //count = 0;
  }

  //===================== LCM HANDLERS =====================================================================================

  void maebot_odometry_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_motor_feedback_t* msg)
  {
    //set the initial tick count for left and right ticks
    static int bark = 0;
    if(init_odo)
    {
      shared.Rticks_1 = msg->encoder_right_ticks;
      shared.Lticks_1 = msg->encoder_left_ticks;
      init_odo = false;
      //count ++;

    }
    else if( new_lidar == true && bark > 4){
      //update the odometry data
      pthread_mutex_lock(&dog);
      shared.Lticks_0 = shared.Lticks_1;
      shared.Rticks_0 = shared.Rticks_1;
      shared.Otime_0 = shared.Otime_1;
      shared.Lticks_1 = msg->encoder_left_ticks;
      shared.Rticks_1 = msg->encoder_right_ticks;
      shared.Otime_1 = msg->utime;
      can_localize = true;
      pthread_mutex_unlock(&dog);
      
      pthread_mutex_lock(&print);
      //cout << "odometry" << endl;
      pthread_mutex_unlock(&print);
      
    }
    else if(new_lidar){
      for(int i = 0; i < int(list_A.size()); i++){
	list_A[i].utime = msg->utime;
      }
      bark++;
    }

  }

  // NO LONGER NEEDED BECAUSE WE HAVE SLAM NOW
  void maebot_pose_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_pose_t* msg)
  {

    pthread_mutex_lock(&dog);
    r_state_r.theta_rob_0 = r_state.theta_rob_1;
    r_state_r.theta_rob_1 = msg->theta;
    r_state_r.timestamp_0 = r_state.timestamp_1;
    r_state_r.timestamp_1 = msg->utime;
    r_state_r.x_rob_0 = r_state.x_rob_1;
    r_state_r.y_rob_0 = r_state.y_rob_1;
    r_state_r.x_rob_1 = msg->x;
    r_state_r.y_rob_1 = msg->y;
    can_update_og = true;
    pthread_mutex_unlock(&dog);

   
    }

  void laser_handler(const lcm::ReceiveBuffer* rbuf, const string& channel, const maebot_laser_scan_t* msg)
  {
    pthread_mutex_lock(&dog);
    shared.num_ranges = msg->num_ranges;
    shared.ranges = msg->ranges;
    shared.thetas = msg->thetas;
    shared.times = msg->times;
    shared.intensities = msg->intensities;
    new_lidar = true;
    pthread_mutex_unlock(&dog);
    
    
  }

  //====================== POSE AND OCCUPANCY GRID FUCNTIONS =============================================================

  //copies shared state to the computational state
  void share2comp(){
    pthread_mutex_lock(&dog);
    comp = shared;
    r_state = r_state_r;
    new_lidar = false;
    can_localize = false;
    pthread_mutex_unlock(&dog);
  }

  void find_pose(){
    share2comp();
    
    //pthread_mutex_lock(&print);
    //cout << "localize" << endl;
    //pthread_mutex_unlock(&print);

    localization();

    //update the the robot's state
    /*
    r_state.theta_rob_0 = r_state.theta_rob_1;
    r_state.theta_rob_1 = list_A[highest_prob_0].theta;
    r_state.timestamp_0 = r_state.timestamp_1;
    r_state.timestamp_1 = list_A[highest_prob_0].utime;
    r_state.x_rob_0 = r_state.x_rob_1;
    r_state.y_rob_0 = r_state.y_rob_1;
    r_state.x_rob_1 = list_A[highest_prob_0].x;
    r_state.y_rob_1 = list_A[highest_prob_0].y;
    */

    //publish our position over LCM
    maebot_pose_t msg_ptr;
    msg_ptr.x = list_A[highest_prob_0].x;
    msg_ptr.y = list_A[highest_prob_0].y;
    msg_ptr.theta = list_A[highest_prob_0].theta;
    msg_ptr.utime = list_A[highest_prob_0].utime;
    cout << "new best x: " << msg_ptr.x << " y: " << msg_ptr.y << endl;
    lcm_inst.publish("MAEBOT_TASK2", &msg_ptr);

    //can_update_og = true;
  }

  //=========localization==============
  void localization(){
    int highest_prob = 0;
    float largest = -FLT_MAX;
    float norm_fact = 0;
    float cnt = 0;
    float cum_prob = list_A[0].prob;
    float inc = float(1.0/float(list_A.size()));
    int index = 0;

    //cout << "old1 tick_l: " << comp.Lticks_0 << " tick_r " << comp.Rticks_0 << endl;
    //cout << "new1 tick_l: " << comp.Lticks_1 << " tick_r " << comp.Rticks_1 << endl;
    
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

	assert(list_A[index].utime != comp.Otime_1);
	//find the position and probability of the new particle, store it in temp
	temp = UpdateFromOdometry(list_A[index].x, list_A[index].y, list_A[index].theta, 
				  list_A[index].utime);
	
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
    
     highest_prob_0 = highest_prob;
     
     //cout << "old2 tick_l: " << comp.Lticks_0 << " tick_r " << comp.Rticks_0 << endl;
     //cout << "new2 tick_l: " << comp.Lticks_1 << " tick_r " << comp.Rticks_1 << endl;

  }

   particle UpdateFromOdometry(float x, float y, float theta, long pose_time)
  {
     particle temp;
     float diff_left = (comp.Lticks_1 - comp.Lticks_0);
     float diff_right = (comp.Rticks_1 - comp.Rticks_0);
     diff_left = (diff_left/480)*M_PI*0.032;
     diff_right = (diff_right/480)*M_PI*0.032;	

     float turn = (diff_right - diff_left)/0.08;
     float distance = (diff_left + diff_right)/2;

     turn = gslu_rand_gaussian (random_number, turn, 0.003 + sqrt(0.1*abs(turn)));
     distance = gslu_rand_gaussian (random_number, distance, sqrt(0.03*abs(distance)));

     float x_change = distance*cos(turn/2 + theta);
     float y_change = distance*sin(turn/2 + theta);

     float theta_change = turn;
	
    // cout << "dx: " << x_change << " dy: " << y_change << " dtheta: " << theta_change << endl; 

     temp.utime = comp.Otime_1;

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
      theta_err = eecs467::wrap_to_pi(w*(shared.times[i]-time_0)/(1000000));

      theta_w = eecs467::wrap_to_pi((eecs467::wrap_to_pi(theta_0 + theta_err)) - shared.thetas[i]);
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

  //finds the error in the robot's position from motion
  void motion_error(long time ){
    r_state.vx = (r_state.x_rob_1 - r_state.x_rob_0)/(r_state.timestamp_1 - r_state.timestamp_0);
    r_state.vy = (r_state.y_rob_1 - r_state.y_rob_0)/(r_state.timestamp_1 - r_state.timestamp_0);
    r_state.w = eecs467::wrap_to_pi(r_state.theta_rob_1 - r_state.theta_rob_0)/(r_state.timestamp_1 - r_state.timestamp_0);

    r_state.x_err = r_state.vx*(time - r_state.timestamp_0);
    r_state.y_err = r_state.vy*(time - r_state.timestamp_0);
    r_state.theta_err = r_state.w*(time - r_state.timestamp_0);
 
  }

  //computation function that updates the occupancy grid
  void compute(){
    //share2comp();
    //pthread_mutex_lock(&print);
    //cout << "occ_grid" << endl;
    //pthread_mutex_unlock(&print);
    //share2comp();
    
    cout << "actual x: " << r_state.x_rob_1 << " y: " << r_state.y_rob_1 << endl;

    float theta_w;
    float x_beam;
    float y_beam;
    eecs467::Point<double> world_coord;
    eecs467::Point<int> grid_coord;
    eecs467::Point<int> beam_end;
    int8_t odds;
    //pthread_mutex_lock(&dog);

    can_update_og = false;
    //iterate through all beams
    for(int i = 0; i < comp.num_ranges; i++){
      //find the x,y coordinates of the endpoint of the lidar beam

      motion_error( comp.times[i] );

      theta_w  = eecs467::wrap_to_pi(eecs467::wrap_to_pi(r_state.theta_rob_0 + r_state.theta_err) - comp.thetas[i]);
      x_beam = comp.ranges[i]*cos(theta_w) + (r_state.x_rob_0 + r_state.x_err);
      y_beam = comp.ranges[i]*sin(theta_w) + (r_state.y_rob_0 + r_state.y_err);
      world_coord.x = double(x_beam); 
      world_coord.y = double(y_beam);
      beam_end = global_position_to_grid_cell( world_coord, OG);

      world_coord.x = double( r_state.x_rob_0 + r_state.x_err );
      world_coord.y = double( r_state.y_rob_0 + r_state.y_err );
      
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

  bool can_loc(){
    bool ret;
    pthread_mutex_lock(&dog);
    ret = can_localize;
    pthread_mutex_unlock(&dog);
    return ret;
  }

  bool update_occ(){
    bool ret;
    pthread_mutex_lock(&dog);
    ret = can_update_og;
    pthread_mutex_unlock(&dog);
    return ret;
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
  eecs467_init (argc, argv);

  occ_grid O;
    
  lcm_inst.subscribe("MAEBOT_POSE", &occ_grid::maebot_pose_handler, &O);
  lcm_inst.subscribe("MAEBOT_LASER_SCAN", &occ_grid::laser_handler, &O);
  lcm_inst.subscribe("MAEBOT_MOTOR_FEEDBACK", &occ_grid::maebot_odometry_handler, &O); 

  //initiate LCM thread
  pthread_t pose_comm;
  pthread_create(&pose_comm, NULL, lcm_comm , NULL);
  pthread_t lidar_comm;
  pthread_create(&lidar_comm, NULL, lcm_comm, NULL);

  while(true){

    if( O.can_loc() ){
      cout << "trying to find pose" << endl;
      O.find_pose();
    }
    if( O.update_occ() ){
      O.compute();
    }
    
  }

  return 0;
}
