#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <common/timestamp.h>

#include "lcmtypes/maebot_motor_command_t.hpp"
#include "lcmtypes/maebot_targeting_laser_command_t.hpp"
#include "lcmtypes/maebot_leds_command_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_laser_scan_t.hpp"
#include "lcmtypes/maebot_occupancy_grid_t.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"

int main(){
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

	srand (time(NULL));
	
    eecs467::OccupancyGrid og(10, 10, 0.05);
    for(int i = 0; i < (int)og.widthInCells(); i++)
    {
        for(int j = 0; j < (int)og.heightInCells(); j++)
        {
            og(i, j) = rand() % 255 - 128;
        }
    }

    maebot_occupancy_grid_t my_data = og.toLCM();

    lcm.publish("OCCUPANCY_GRID_GUI", &my_data);

    std::cout << "sent" << std::endl;

    return 0;
}
