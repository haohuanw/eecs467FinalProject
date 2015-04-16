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


class LCMHandler
{
    private:
        lcm::LCM *lcm_;

    public:
        LCMHandler(lcm::LCM *lcm_t) : lcm_(lcm_t){}
        ~LCMHandler(){}

        void handleMessage1(const lcm::ReceiveBuffer *rbuf,
                const std::string& channel,
                const maebot_pose_t *msg)
        {
            //std::cout << "Laser: " << msg->utime << std::endl;
            srand (time(NULL));
/*
            eecs467::OccupancyGrid og(10, 10, 0.05);
            for(int i = 0; i < (int)og.widthInCells(); i++)
            {
                for(int j = 0; j < (int)og.heightInCells(); j++)
                {
                    og(i, j) = rand() % 255 - 128;
                }
            }

            maebot_occupancy_grid_t my_data = og.toLCM();

            lcm_->publish("OCCUPANCY_GRID_GUI", &my_data);
  */          
            maebot_pose_t tmp;

            tmp.utime = msg->utime;
            tmp.theta = msg->theta;
            tmp.x = msg->x * 1.5;
            tmp.y = msg->y * 1.5;

            lcm_->publish("MAEBOT_POSE_GUI_E", &tmp);

/*            for(int i = 0; i < 1000; ++i){
                tmp.x = msg->x + (rand() % 300 / 100.);
                tmp.y = msg->y + (rand() % 300 / 100.);
                lcm_->publish("MAEBOT_PARTICLE_GUI", &tmp);
            } 
   */     }

        void handleMessage2(const lcm::ReceiveBuffer *rbuf,
                const std::string& channel,
                const maebot_motor_feedback_t *msg)
        {
            std::cout << "Feedback: " << msg->utime << std::endl;
        }
};

int main(){
    lcm::LCM lcm;
    if(!lcm.good())
        return 1;

    LCMHandler handler(&lcm);

    lcm.subscribe("MAEBOT_POSE",
            &LCMHandler::handleMessage1,
            &handler);
    //lcm.subscribe("MAEBOT_MOTOR_FEEDBACK",
    //                      &LCMHandler::handleMessage2,
    //                      &handler);

    while(lcm.handle() == 0);
    return 0;
}
