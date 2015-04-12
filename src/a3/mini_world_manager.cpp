#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <stdlib.h>
#include <lcm/lcm-cpp.hpp>
#include <signal.h>
#include <math.h>
#include <string>
#include <deque>

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/matd.h"
#include "math/math_util.h"
#include "math/point.hpp"
#include "imagesource/image_util.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/ui_dest_list_t.hpp"
#include "lcmtypes/bot_commands_t.hpp"
#include "maebot_gui_t.hpp"
#include "Navigator.hpp"

int main(){
    lcm::LCM lcm;
    bot_commands_t cmd;
    cmd.x_rob = 0.0;
    cmd.y_rob = 0.0;
    cmd.theta_rob = 0.0;
    cmd.x_dest = 3.0;
    cmd.y_dest = 0.0;
    lcm.publish("MAEBOT_PID_COMMAND_RED",&cmd);
    std::cout<<"publish to the MAEBOT_PID_COMMAND_RED"<<std::endl;
    usleep(2000000);
    cmd.x_rob = 0.0;
    cmd.y_rob = 0.0;
    cmd.theta_rob = 0.0;
    cmd.x_dest = 0.0;
    cmd.y_dest = 0.0;
    lcm.publish("MAEBOT_PID_COMMAND_RED",&cmd);
    std::cout<<"publish a stop override"<<std::endl; 
}
