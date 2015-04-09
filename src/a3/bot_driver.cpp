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

  	move_data(){
  		speed = 0;
  		x_rob = y_rob = x_dest = y_dest = theta_rob = theta_dest = 0;

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

	//LCM handler for reading in maebot commands from the master program
	/*void bot_commands_handler(const lcm::ReceiveBuffer* rbuf, const string &channel, const bot_commands_t* msg){
		pthread_mutex_lock(&comms_m);
		read.speed = msg->speed;
		read.x_rob = msg->x_rob;
		read.y_rob = msg->y_rob;
		read.theta_rob = msg->theta_rob;

		read.x_dest = msg->x_dest;
		read.y_dest = msg->y_dest;
		read.theta_dest = msg->theta_dest;
		pthread_mutex_unlock(&comms_m);
	}*/

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

			//cout << "theta: " << read.theta_rob << endl;

			
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
		pthread_mutex_lock(&comms_m);
		double angle = read.theta_rob;
		double angle_2 = read.theta_rob;
		pthread_mutex_unlock(&comms_m);

		maebot_motor_command_t msg;
	    maebot_motor_command_t *msg_ptr = &msg; 

		//run until the robot has turned 90 degrees
		while( abs(180*(angle_2 - angle)/M_PI) <= 65){


			pthread_mutex_lock(&comms_m);
			cout << "angle delta: " <<  abs(180*eecs467::wrap_to_pi(angle_2 - angle)/M_PI) << endl;
			angle_2 = read.theta_rob;
			pthread_mutex_unlock(&comms_m);

			msg.motor_left_speed =  -0.15;
			msg.motor_right_speed = 0.16;


			lcm_inst.publish("MAEBOT_MOTOR_COMMAND", &msg);
			usleep(5000);


		}

		pthread_mutex_lock(&comms_m);
		cout << "angle delta: " <<  abs(180*eecs467::wrap_to_pi(angle_2 - angle)/M_PI) << endl;
		angle_2 = read.theta_rob;
		pthread_mutex_unlock(&comms_m);


		msg.motor_left_speed =  0.0;
	    msg.motor_right_speed = 0.0;
	    lcm_inst.publish("MAEBOT_MOTOR_COMMAND", msg_ptr);
	    //usleep(5000);
	    //return true;
	}

	void go_straight(){

		uint64_t time;
		maebot_motor_command_t msg;
	    maebot_motor_command_t *msg_ptr = &msg; 

	    pthread_mutex_lock(&comms_m);
	    double x_origin = read.x_rob;
	    double y_origin = read.y_rob;
	    double theta_origin = read.theta_rob;
	    pthread_mutex_unlock(&comms_m);

	    double x_current = x_origin;
	    double y_current = y_origin;
	    double theta_current = theta_origin;

	    double x_dest = 1.5;
	    double y_dest = 0.0;
	    double theta_dest = 0.0;

	    double left_speed =  0.25;
	    double right_speed = 0.25;

	    const double MAX_SPEED = 0.3;
	    const double MIN_SPEED = 0.16;
	    const double MIN_GAIN = 0.0004;
	    const double MAX_GAIN = 0.0008;

	    
	    while(x_dest > x_current){

	    	cout << "theta_current: " << 180*theta_current/M_PI << endl;
	    	cout <<"x_current: " << x_current << endl;
	    	cout <<"y_current: " << y_current << endl;

	    	//IF WE'RE ON THE LEFT SIDE OF THE LANE, CORRECT BY MOVING RIGHT
	    	if( y_current - y_origin > 0.04 && abs(180*eecs467::wrap_to_pi(theta_origin - theta_dest)/M_PI) < 5 ){
	    		cout << "far left" << endl;
	    		if(left_speed < MAX_SPEED)
	    			left_speed += MIN_GAIN;
	    		else if(right_speed > MIN_SPEED)
	    			right_speed -= MIN_GAIN;
		    	
			}
			//IF WE'RE ON THE RIGHT SIDE OF THE LANE, CORRECT BY MOVING LEFT
			else if( y_current - y_origin < -0.04 && abs(180*eecs467::wrap_to_pi(theta_origin - theta_dest)/M_PI) < 5) {

				cout << "far right" << endl;
	    		if(left_speed > MIN_SPEED)
		    		left_speed -= MIN_GAIN;
		    	else if( right_speed < MAX_SPEED)
		    		right_speed += MIN_GAIN;
		    	
			}
			//IF WE'RE VEERING RIGHT, CORRECT BY GOING LEFT
			if( (180*eecs467::wrap_to_pi(theta_current - theta_dest)/M_PI < -5.0) ) {
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

		    	if( y_current - y_origin > 0.004 ){
		    		cout << "straight left" << endl;
		    			left_speed = 0.175;
	    				right_speed = 0.165;
			    	
				}
				//IF WE'RE ON THE RIGHT SIDE OF THE LANE, CORRECT BY MOVING LEFT
				else if( y_current - y_origin < -0.004 ) {

					cout << "straight right" << endl;
		    			left_speed = 0.165;
	    				right_speed = 0.175;
			    	
				}
	    	}


	    	msg_ptr->motor_left_speed = left_speed;
	    	msg_ptr->motor_right_speed = right_speed;

	    	//cout << ""

	    	lcm_inst.publish("MAEBOT_MOTOR_COMMAND", msg_ptr);
	    	usleep(50000);

	    	pthread_mutex_lock(&comms_m);
	    	x_current = read.x_rob;
	    	y_current = read.y_rob;
	    	theta_current = read.theta_rob;
	    	pthread_mutex_unlock(&comms_m);

	    }
	    


	    /*
		for(int i = 0; i < 200; i++){
			//maebot 4 full battery values
			msg.motor_left_speed =  0.2;
	    	msg.motor_right_speed = 0.193;
	    	lcm_inst.publish("MAEBOT_MOTOR_COMMAND", msg_ptr);
	    	usleep(50000);
	    	cout << " hi " << endl;

		}*/



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

	C.go_straight();

	/*while(true){
		C.turn_90();
		usleep(5000000);
	}*/

	return 0;
}
