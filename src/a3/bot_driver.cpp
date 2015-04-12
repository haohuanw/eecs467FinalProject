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

  	bool turn;

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

	bool do_something(){
		bool ret;
		pthread_mutex_lock(&comms_m);
		ret = new_command;
		comp = read;
		pthread_mutex_unlock(&comms_m);
		

		return ret;
	}

	//LCM handler for reading in maebot commands from the master program
	void bot_commands_handler(const lcm::ReceiveBuffer* rbuf, const string &channel, const bot_commands_t* msg){
		pthread_mutex_lock(&comms_m);
		read.x_rob = msg->x_rob;
		read.y_rob = msg->y_rob;
		read.theta_rob = msg->theta_rob;

		read.x_dest = msg->x_dest;
		read.y_dest = msg->y_dest;
		// read.theta_dest = msg->theta_dest;
		if(x_dest > x_rob){
			read.theta_dest = 0;
		}
		else if(x_dest < x_rob){
			read.theta_dest = -M_PI;
		}
		else if(y_dest > y_rob){
			read.theta_dest = M_PI/2.0;
		}
		else{
			read.theta_dest = -M_PI/2.0;
		}

		new_command = true;
		pthread_mutex_unlock(&comms_m);
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

			cout << "RED===: ";
			cout << "x_rob: " << read.x_rob << " y_rob: " << read.y_rob << " theta_rob: " << read.theta_rob << endl;

			
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
	void turn_X(double theta_current, double theta_dest){

		double angle = theta_current;
		double angle_2 = theta_dest;

		if(angle - angle2 > 0){
				msg.motor_left_speed =  0.18;
				msg.motor_right_speed = -0.18;
		}
		else{
				msg.motor_left_speed =  -0.18;
				msg.motor_right_speed = 0.18;
		}

		pthread_mutex_lock(&comms_m);
		read.theta_rob = theta_current;
		pthread_mutex_unlock(&comms_m);

		maebot_motor_command_t msg;
	    maebot_motor_command_t *msg_ptr = &msg; 
	    bot_commands_t bot_msg;
	    bot_commands_t *bot_msg_ptr = &bot_msg; 

	    double angle_diff = 0;

		//run until the robot has turned 90 degrees
		while( true ){


			pthread_mutex_lock(&comms_m);
			cout << "angle diff: " <<  angle_diff << endl;
			angle = read.theta_rob;
			pthread_mutex_unlock(&comms_m);
			angle_diff = 180*eecs467::wrap_to_pi(angle_2 - angle)/M_PI;
			//end = time(0);

			if(abs(angle_diff) > 0.5 ){

				usleep(5000);
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
				usleep(2000);

				break;
			}

			

		}
		
		cout << "angle delta: " <<  abs(180*eecs467::wrap_to_pi(angle_2 - angle)/M_PI) << endl;
		


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

		bot_msg_ptr->reach_dest = true;
	    lcm_inst.publish("MAEBOT_PID_FEEDBACK_RED", bot_msg_ptr);


		return 0;
	}

	void go_straight(double theta_dest, double x_dest, double y_dest, double theta_in, double x_in, double y_in){

		maebot_motor_command_t msg;
	    maebot_motor_command_t *msg_ptr = &msg; 
	    bot_commands_t bot_msg;
	    bot_commands_t *bot_msg_ptr = &bot_msg; 

	    double theta_current = theta_in;
	    double path_pos, lane_pos;
	    double path_dest, lane_dest;


	    double left_speed =  0.17;
	    double right_speed = 0.163;

	    const double MAX_SPEED = 0.19;
	    const double MIN_SPEED = 0.16;
	    const double MIN_GAIN = 0.0004;
	    const double MAX_GAIN = 0.0008;


	    pthread_mutex_lock(&comms_m);
	   	read.x_rob = path_pos;
	   	read.y_rob = lane_pos;
		read.theta_rob = theta_current;	    
		pthread_mutex_unlock(&comms_m);

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
	    else if(theta_dest == -M_PI/2.0){
	    	path_pos = y_in;
	    	lane_pos = x_in;
	    	path_dest = y_dest;
	    	lane_dest = x_dest;
	    }
	    else{}

	    double angle_diff;

	    while( not_done(theta_dest, path_pos, path_dest) ){

	    	cout << "theta_current: " << 180*theta_current/M_PI << endl;
	    	cout <<"path_position: " << path_pos << endl;
	    	cout <<"lane_position: " << lane_pos << endl;
	    	//BACKUP P controller works


	    	//IF WE'RE ON THE LEFT SIDE OF THE LANE, CORRECT BY MOVING RIGHT
	    	if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == 1 && abs(180*eecs467::wrap_to_pi(theta_current - theta_dest)/M_PI) < 5 ){
	    		cout << "far left" << endl;
	    		if(left_speed < MAX_SPEED)
	    			left_speed += MIN_GAIN;
	    		else if(right_speed > MIN_SPEED)
	    			right_speed -= MIN_GAIN;
		    	
			}
			//IF WE'RE ON THE RIGHT SIDE OF THE LANE, CORRECT BY MOVING LEFT
			else if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == -1 && abs(180*eecs467::wrap_to_pi(theta_current - theta_dest)/M_PI) < 5) {

				cout << "far right" << endl;
	    		if(left_speed > MIN_SPEED)
		    		left_speed -= MIN_GAIN;
		    	else if( right_speed < MAX_SPEED)
		    		right_speed += MIN_GAIN;
		    	
			}
			//IF WE'RE VEERING RIGHT, CORRECT BY GOING LEFT
			else if( (180*eecs467::wrap_to_pi(theta_current - theta_dest)/M_PI < -5.0) ) {
					cout << "veering right" << endl;
		
		    		if(left_speed > MIN_SPEED)
		    			left_speed -=MAX_GAIN;
		    		else if( right_speed < MAX_SPEED)
		    			right_speed += MAX_GAIN;
		    }
		    //IF WE"RE VEERING LEFT, CORRECT BY GOING RIGHT
	    	else if(180*eecs467::wrap_to_pi(theta_current - theta_dest)/M_PI > 5.0 ){
	    		cout << "veering left" << endl;
	    		if(left_speed < MAX_SPEED)
	    			left_speed += MAX_GAIN;
	    		else if(right_speed > MIN_SPEED)
	    			right_speed -= MAX_GAIN;
	    	}
	    	else{

	    		left_speed = 0.17;
	    		right_speed = 0.163;

		    	if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == 1 ){
		    		cout << "straight left" << endl;
		    			left_speed = 0.175;
	    				right_speed = 0.165;
			    	
				}
				//IF WE'RE ON THE RIGHT SIDE OF THE LANE, CORRECT BY MOVING LEFT
				else if( too_far(theta_dest, lane_pos, lane_dest, 0.04) == -1 ) {

					cout << "straight right" << endl;
		    			left_speed = 0.165;
	    				right_speed = 0.175;
			    	
				}
	    	}

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
	    bot_msg_ptr->reach_dest = true;
	    lcm_inst.publish("MAEBOT_PID_FEEDBACK_RED", bot_msg_ptr);


	}

};



int main(int argc, char *argv[]){
  //eecs467_init (argc, argv);

	//init LCM
	Motion_Class C;
	//lcm_inst.subscribe("BOT_COMMANDS", &Motion_Class::bot_commands_handler, &C);
	lcm_inst.subscribe("MAEBOT_MOTOR_FEEDBACK", &Motion_Class::odometry_handler, &C);
	pthread_t odometry_comm;
	pthread_create(&odometry_comm, NULL, lcm_comm, NULL);

	//C.go_straight(0.0, 1.5, 0.0, 0.0, 0.0, 0.0);

	while(true){
		if(C.do_something()){
			if(comp.turnning == true)
				turn_X(comp.theta_rob, comp.theta_dest);
			else{
				go_straight(comp.theta_dest, comp.x_dest, comp.y_dest, comp.theta_rob, comp.x_rob, comp.y_rob);
			}
		}
		
	}

	return 0;
}