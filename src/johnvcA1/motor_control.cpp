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


int main(){



  return 0;
}
