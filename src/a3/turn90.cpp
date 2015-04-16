/*prototype for controlling robot motion*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <ctime>


// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"
#include "math/point.hpp"

#include "apps/eecs467_util.h"    // This is where a lot of the internals live

//lcm
#include "lcm/lcm-cpp.hpp"
//#include "lcmtypes/bot_commands_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/bot_commands_t.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "math/angle_functions.hpp"
#include "math.h"

using namespace std;

//============= LCM SETUP ====================
pthread_mutex_t comms_m;
lcm::LCM lcm_inst;

void *lcm_comm(void *input){
	while(true){

		lcm_inst.handle();
	}
	return NULL;
}


//============= USEFUL DATA STRUCTURES ===========
struct odo_data{
	int32_t left_ticks;
	int32_t right_ticks;


};


struct move_data{
	uint8_t speed;

	double x_rob;
  	double y_rob;
  	double theta_rob;
  
  	//destination pose
  	double x_dest;
  	double y_dest;
  	double theta_dest;

  	double turn;

  	move_data(){
  		speed = 0;
  		x_rob = y_rob = x_dest = y_dest = theta_rob = theta_dest = 0;
  		turn = false;

  	}

};


//================ MAIN MOTION CLASS ===============
struct Motion_Class{
	move_data read;
	move_data comp;
	odo_data o_read;
	bool first_read;
	bool new_odo, new_command;


	Motion_Class(){
		new_odo = false;
		first_read = true;
	}
	//LCM handler for reading in odometry data from the maebot itself to have some control
	//between bot command messages

	//might want to make the threads, locks, and general data management 
	//better later but fuck it for now
	void odometry_handler(const lcm::ReceiveBuffer* rbuf, const string &channel, const maebot_motor_feedback_t* msg){
		pthread_mutex_lock(&comms_m);
		//cout << "in odo" << endl;
		if(!first_read){
			double d_L = (msg->encoder_left_ticks - o_read.left_ticks)*0.032*M_PI/480.0;
			double d_R = (msg->encoder_right_ticks - o_read.right_ticks)*0.032*M_PI/480.0;

			double ticks_avg = double(d_L + d_R )/2.0;
			double d_theta = eecs467::wrap_to_pi(double(d_R - d_L)/0.08);
			double a = eecs467::wrap_to_pi(d_theta/2.0);


			
			read.x_rob = cos(eecs467::wrap_to_pi(read.theta_rob + a))*ticks_avg + read.x_rob;
			read.y_rob = sin(eecs467::wrap_to_pi(read.theta_rob + a))*ticks_avg + read.y_rob;
			read.theta_rob = eecs467::wrap_to_pi(d_theta + read.theta_rob);

			//cout << "x_rob: " << read.x_rob << " y_rob: " << read.y_rob << " theta_rob: " << read.theta_rob << endl;

			
		}

		o_read.left_ticks = msg->encoder_left_ticks;
		o_read.right_ticks = msg->encoder_right_ticks;
		new_odo = true;
		first_read = false;
		pthread_mutex_unlock(&comms_m);
	}


	/*90 degree turn handler psuedocode*/
	//parameters to tune: 
	//1-angle cutoff
	//2-average speed
	//3- different in motors
	void turn_90(){

		maebot_motor_command_t msg;
	    maebot_motor_command_t *msg_ptr = &msg; 

	    pthread_mutex_lock(&comms_m);
		double init_angle = read.theta_rob;
		

		pthread_mutex_unlock(&comms_m);
		double angle_diff_dest = 90.0;
		double speed = 0.08;
		double angle_diff = 0;

		//run until the robot has turned 90 degrees
		while( true ){


			pthread_mutex_lock(&comms_m);
			angle_diff = 180*eecs467::angle_diff(read.theta_rob, init_angle)/M_PI;
			cout << "angle diff: " <<  angle_diff << endl;
			pthread_mutex_unlock(&comms_m);

			if(angle_diff < 0.9*angle_diff_dest ){

				usleep(5000);
				msg.motor_left_speed =  -speed*((angle_diff_dest - angle_diff)/abs(angle_diff_dest))- 0.11;
				msg.motor_right_speed = speed*((angle_diff_dest - angle_diff)/abs(angle_diff_dest)) + 0.11;
				lcm_inst.publish("MAEBOT_MOTOR_COMMAND", &msg);
				

				/*angle_diff = abs(angle_2-angle);
				if(angle_diff > 3*M_PI/4){
					angle_diff = (M_PI-angle) + abs(-M_PI - angle_2);
				}
				angle_diff = 180*angle_diff/M_PI;*/
			}
			else{
				msg.motor_left_speed =  0;
				msg.motor_right_speed = 0;


				lcm_inst.publish("MAEBOT_MOTOR_COMMAND", &msg);
				break;
			}

			

		}
		
		//cout << "angle delta: " <<  angle_diff << endl;
		


		msg.motor_left_speed =  0.0;
	    msg.motor_right_speed = 0.0;
	    lcm_inst.publish("MAEBOT_MOTOR_COMMAND", msg_ptr);

	    //usleep(5000);
	    //return true;
	}

	//FUNCTION DETERMINES IF WE STILL HAVE SOMEWHERE TO GO
	bool not_done(double theta_dest, double path_pos, double path_dest){

		//if moving along positive x,y
		if(theta_dest == 0 || theta_dest == M_PI/2.0){
			if(path_pos < path_dest)
				return true;
		}
		else{
			//if moving along negative x,y
			if(path_dest < path_pos)
				return true;
		}
		return false;
	}

	//returns -1 if too far right, returns 1 if too far left, 0 if ok
	int too_far(double theta_dest, double lane_pos, double lane_dest, double thresh){

		if(theta_dest == 0 || theta_dest == M_PI/2.0){
			if(lane_pos-lane_dest > thresh)
				return 1;
			else if(lane_pos-lane_dest < -thresh)
				return -1;
			else
				return 0;
		}
		else{
			if(lane_pos-lane_dest > thresh)
				return -1;
			else if(lane_pos-lane_dest < -thresh)
				return 1;
			else
				return 0;
		}


		return 0;
	}

	void go_straight(double theta_dest, double x_dest, double y_dest, double theta_in, double x_in, double y_in){

		maebot_motor_command_t msg;
	    maebot_motor_command_t *msg_ptr = &msg; 
	    //bot_commands_t bot_msg;
	    //bot_commands_t *bot_msg_ptr = &bot_msg; 

	    double theta_current = theta_in;
	    double path_pos, lane_pos;
	    double path_dest, lane_dest;
	    double lane_error = 0, theta_error = 0;

	    double base =  0.17;

	    const double MAX_GAIN = 0.10;
	    const double MIN_GAIN = 0.05;
	    const double L_MAX = 0.03;
	    const double L_MIN = 0.05;
	    double left_speed, right_speed;


	    //REMEMBER: x in the robot's frame is the direction of motion, always!
	    //HOWEVER: x in the world frame is static

	    //ORIENT THE STARTING POSITION AND DESTINATION ACCORDINGLY

	    //If moving along the positive x axis
	    if(theta_dest == 0){
	    	path_pos = x_in;
	    	lane_pos = y_in;
	    	path_dest = x_dest;
	    	lane_dest = y_dest;

	    }
	    //if moving along the positive y axis
	    else if(theta_dest == M_PI/2.0){
	    	path_pos = y_in;
	    	lane_pos = x_in;
	    	path_dest = y_dest;
	    	lane_dest = x_dest;

	    }
	    //if moving along the negative x axis
	    else if(theta_dest == M_PI || theta_dest == -M_PI){
	    	path_pos = x_in;
	    	lane_pos = y_in;
	    	path_dest = x_dest;
	    	lane_dest = y_dest;

	    }
	    //if moving along the negative y axis
	    else{
	    	path_pos = y_in;
	    	lane_pos = x_in;
	    	path_dest = y_dest;
	    	lane_dest = x_dest;
	    }

	    pthread_mutex_lock(&comms_m);
	   	read.x_rob = path_pos;
	   	read.y_rob = lane_pos;
		read.theta_rob = theta_current;	    
		pthread_mutex_unlock(&comms_m);

	    while( not_done(theta_dest, path_pos, path_dest) ){

	    	cout << "theta_current: " << 180*theta_current/M_PI << endl;
	    	cout <<"path_position: " << path_pos << endl;
	    	cout <<"lane_position: " << lane_pos << endl;
	    	//BACKUP P controller works

	    	lane_error += lane_pos - lane_dest;
	    	theta_error += theta_current - theta_dest;

	    	
	    	//IF WE'RE ON THE LEFT SIDE OF THE LANE, CORRECT BY MOVING RIGHT
	    	/*
	    	if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == 1 && abs(180*eecs467::angle_diff(theta_current, theta_dest)/M_PI) < 3 ){
	    		cout << "far left" << endl;
	    		left_speed = base + MIN_GAIN*(lane_pos - lane_dest)/0.15;
	    	    right_speed = base;
	    		
		    	
			}
			//IF WE'RE ON THE RIGHT SIDE OF THE LANE, CORRECT BY MOVING LEFT
			else if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == -1 && abs(180*eecs467::angle_diff(theta_current,theta_dest)/M_PI) < 3) {
				cout << "far right" << endl;
				left_speed = base;
	    	    right_speed = base + MIN_GAIN*(lane_pos - lane_dest)/0.15;
	    		
		    	
			}
			//IF WE'RE VEERING RIGHT, CORRECT BY GOING LEFT
			else if( (180*eecs467::angle_diff(theta_current, theta_dest)/M_PI < -5.0) ) {
				cout << "veering right" << endl;
				left_speed = base;
	        	right_speed = base + MAX_GAIN*(180*abs(eecs467::angle_diff(theta_current, theta_dest)/M_PI))/50.0;
		
		  
		    }
		    //IF WE"RE VEERING LEFT, CORRECT BY GOING RIGHT
	    	else if(180*eecs467::angle_diff(theta_current, theta_dest)/M_PI > 5.0 ){
	    		cout << "veering left" << endl;
	    		left_speed = base + MAX_GAIN*(180*abs(eecs467::angle_diff(theta_current, theta_dest)/M_PI))/50.0;
	    	    right_speed = base;
	    		
	    	}
	    	else{
	    		left_speed = base;
	    		right_speed = base;
	    	}
			*/

			left_speed = base + MAX_GAIN*(180*eecs467::angle_diff(theta_current, theta_dest)/M_PI)/180.0 + L_MIN*(lane_pos - lane_dest)/0.15; 
			right_speed = base - MAX_GAIN*(180*eecs467::angle_diff(theta_current, theta_dest)/M_PI)/180.0 - L_MIN*(lane_pos - lane_dest)/0.15 ;




	    	//if(left_speed >= 0.24)
	    	//	left_speed = 0.22;
	    	//if(right_speed >= 0.24)
	    	//	right_speed = 0.22;

	    	msg_ptr->motor_left_speed = left_speed;
	    	msg_ptr->motor_right_speed = right_speed;


	    	lcm_inst.publish("MAEBOT_MOTOR_COMMAND", msg_ptr);
	    	usleep(50000);

	    	pthread_mutex_lock(&comms_m);
	    	path_pos = read.x_rob;
	    	lane_pos  = read.y_rob;
	    	theta_current = read.theta_rob;
	    	pthread_mutex_unlock(&comms_m);

	    }
	   

	    //tell world manager we're done
	    //bot_msg_ptr->reach_dest = 1;
	    //lcm_inst.publish("MAEBOT_PID_FEEDBACK_RED", bot_msg_ptr);


	}

};

int main(){
	Motion_Class C;
	lcm_inst.subscribe("MAEBOT_MOTOR_FEEDBACK", &Motion_Class::odometry_handler, &C);
	pthread_t odometry_comm;
	pthread_create(&odometry_comm, NULL, lcm_comm, NULL);

	//while(true){
	//C.turn_90();
	C.go_straight(0, 1.5, 0, 0, 0, 0);
	//}


}