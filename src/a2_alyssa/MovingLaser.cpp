#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <string>
#include <vector>
#include <iostream>
#include <cassert>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"

#include "MovingLaser.hpp"

#define testing 0

using namespace std;
//Creates and returns a LaserScan and fills its origins vector.
LaserScan MovingLaser::findOrigin(LaserScanRange approx_scan)
{
	LaserScan ls;
    if(testing) { cout << "start time: " << approx_scan.start_pose.utime << "\nend time: " << approx_scan.end_pose.utime << "\n"; }
	for(unsigned int i = 0; i < approx_scan.scan.times.size(); i++)
    {
        if(testing) { cout << "Laser scan " << i << endl; }
		ls.origins.push_back(findOriginSingle(approx_scan.scan.times[i],
			approx_scan.start_pose, approx_scan.end_pose));
    }
	
	ls.scan = approx_scan.scan;
    ls.valid = true;
    ls.end_pose = approx_scan.end_pose;
	
	return ls;
}

//Returns a new pose corresponding to the interpolated pose between a and b at time t.
maebot_pose_t MovingLaser::findOriginSingle(int64_t t, maebot_pose_t a, maebot_pose_t b)
{
	/*if(t < a.utime || t > b.utime)
		cout << "Out of range! t: " << t << ", a: " << a.utime << ", b: " << b.utime << endl;*/

    assert(b.utime >= a.utime);
	double percent = (t - (double)a.utime) / ((double)b.utime - (double)a.utime);
	maebot_pose_t n;
	n.x = (b.x - a.x) * percent + a.x;
	n.y = (b.y - a.y) * percent + a.y;
	n.theta = eecs467::wrap_to_2pi(eecs467::angle_diff(eecs467::wrap_to_2pi(b.theta), eecs467::wrap_to_2pi(a.theta)) * percent + eecs467::wrap_to_2pi(a.theta));
	n.utime= t;
    if(testing) { cout << "\tx coord: " << n.x << "\n\ty coord: " << n.y << "\n\ttheta: " << n.theta << "\n\ttime: " << n.utime << endl; }
	return n;
}
