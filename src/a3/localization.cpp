#include <gtk/gtk.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <lcm/lcm-cpp.hpp>
#include <vector>
#include <deque>
#include <iostream>

#include "common/getopt.h"
#include "common/timestamp.h"
#include "common/timestamp.h"
#include "imagesource/image_u32.h"
#include "imagesource/image_util.h"
#include "lcmtypes/maebot_motor_feedback_t.hpp"
#include "lcmtypes/maebot_sensor_data_t.hpp"
#include "particle_data.hpp"
#include "action_model.hpp"
#include "pose_tracker.hpp"

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include <math/point.hpp>
#include "occupancy_map.hpp"
#include <math/gsl_util_rand.h>

static const char* MAP_TO_READ;


/*
******TO DO LIST *******
pick localization (our/johns)
feed in correct map
remove unnecessary printing and displays
lcm command/communicate with world manager
*/


class state_t
{
    public:
        occupancy_map map;

        lcm::LCM lcm;
        pthread_t lcm_thread_pid;

        pthread_mutex_t data_mutex;
        pthread_mutex_t run_mutex;

        particle_data particles;

        std::vector<maebot_pose_t> our_path;
        std::vector<maebot_pose_t> collins_path;
        maebot_pose_t curr_collin_pose;

        pthread_mutex_t mutex; // for accessing the arraysf
        image_u8_t *image_buf;
        // FILE *pose_fp;
        // FILE *odo_fp;
        // FILE *error_fp;


    public:
        state_t()
        {
            //initialize particles at (0,0,0)
            maebot_pose_t temp;
            temp.x=0;
            temp.y=0;
            temp.theta=0;

            //read_map();
            map = occupancy_map(5.0,5.0,0.05,1.0);
            particles = particle_data(1000, temp, &map.grid);
            read_map();
            gslu_rand_seed(); 
            //action_error_model = action_model();
            //bot_tracker = pose_tracker();

            if (pthread_mutex_init(&run_mutex, NULL)) {
                printf("run mutex init failed\n");
                exit(1);
            }
            if (pthread_mutex_init(&data_mutex, NULL)) {
                printf("pose_curr mutex init failed\n");
                exit(1);
            }

            image_buf = nullptr;

            lcm.subscribe("MAEBOT_POSE", &state_t::pose_handler, this);
            lcm.subscribe("MAEBOT_MOTOR_FEEDBACK", &state_t::odo_handler, this);
            lcm.subscribe("MAEBOT_LASER_SCAN", &state_t::laser_scan_handler, this);
            // pose_fp = fopen("pose_data.txt","w");
            // odo_fp = fopen("odo_data.txt","w");
            // error_fp = fopen("error_data.txt","w");

        }

        ~state_t()
        {
            pthread_mutex_destroy(&mutex);
            pthread_mutex_destroy(&run_mutex);
            pthread_mutex_destroy(&data_mutex);
            image_u8_destroy(image_buf);
        }

        void pose_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_pose_t *msg){
            pthread_mutex_lock(&data_mutex);

            collins_path.push_back(*msg);
            curr_collin_pose = *msg;
            //fprintf(pose_fp,"%lld %f %f\n",msg->utime,msg->x,msg->y);
            pthread_mutex_unlock(&data_mutex);
        }

        void odo_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_motor_feedback_t *msg)
        {
            //printf("odo handle\n");
            pthread_mutex_lock(&data_mutex);
            //printf("start pushing\n");
            //bot_tracker.push_msg(*msg, action_error_model);
            particles.push_odo(*msg);
            pthread_mutex_unlock(&data_mutex);

            pthread_mutex_lock(&data_mutex);
            if(particles.ready()){
                //printf("particle filter process\n");
                particles.update();
                //printf("best particle: %f %f\n",particles.get_best().x,particles.get_best().y);
                maebot_pose_t best = particles.get_best();
                our_path.push_back(best);
            }  

            /*
            	WM_COMM_t comm_msg;
            	maebot_pose_t loc;
				lcm.publish("WM_COMM", &comm_msg);

			*/
				
            pthread_mutex_unlock(&data_mutex);
        }

        void laser_scan_handler (const lcm::ReceiveBuffer* rbuf, const std::string& channel,const maebot_laser_scan_t *msg)
        {
            pthread_mutex_lock(&data_mutex);
            //printf("laser handle\n");
            if(particles.processing == false){
                particles.push_scan(*msg);

            } 
            pthread_mutex_unlock(&data_mutex);
        }









        void init_thread()
        {
            pthread_create(&lcm_thread_pid,NULL,&state_t::run_lcm,this);
        }

        static void* run_lcm(void *input)
        {
            state_t* state = (state_t*) input;
            while(1){
                state->lcm.handle();
            }
            return NULL;
        }


        static uint8_t to_grayscale(int8_t logOdds)
        {
            return 127 - logOdds;
        }

        static void render_grid(state_t * state)
        {
            eecs467::OccupancyGrid& grid = state->map.get_grid();
            if(state->image_buf == nullptr){
                state->image_buf = image_u8_create(grid.widthInCells(),grid.heightInCells());
            }
            for (size_t y = 0; y < grid.heightInCells(); y++)
            {
                for (size_t x = 0; x < grid.widthInCells(); x++)
                {
                    state->image_buf->buf[(y * state->image_buf->stride) + x] = to_grayscale(grid.logOdds(x,y));
                }
            }
        }

        static void save_map(state_t *state)
        {
            FILE *fp;
            fp = fopen("empty_map.txt","w");
            eecs467::OccupancyGrid& grid = state->map.get_grid();
            fprintf(fp,"%d\n",grid.heightInCells());
            fprintf(fp,"%d\n",grid.widthInCells());
            for(size_t y = 0; y < grid.heightInCells();y++){
                for(size_t x = 0; x < grid.widthInCells(); x++){
                    fprintf(fp,"%d\n",grid.logOdds(x,y));
                }
            }
            fclose(fp);
        }

        void read_map()
        {
            FILE *fp;
            uint8_t temp;
            fp = fopen(MAP_TO_READ,"r");
            fscanf(fp,"%d\n",&temp);
            if(temp != map.grid.heightInCells()){
                std::cout << "Height not match\n";
                exit(1);
            }
            fscanf(fp,"%d\n",&temp);
            if(temp != map.grid.widthInCells()){
                std::cout << "Width not match\n";
                exit(1);
            }
            map = occupancy_map(5.0,5.0,0.05,1.0); //if s_rate changes here, correction in samples class needs adjustment
            for(size_t y = 0; y < map.grid.heightInCells();y++){
                for(size_t x = 0; x < map.grid.widthInCells(); x++){
                    fscanf(fp,"%d ",&temp);
                    map.grid.setLogOdds(x,y,temp);
                }
            }
            fclose(fp);
        }



};

int main(int argc, char ** argv)
{
    MAP_TO_READ = argv[1];
    printf("%s",MAP_TO_READ);
    state_t state;
    state.init_thread();
    //comment below disable the vx
    //state.draw(&state,state.world);
    gdk_threads_init();
    gdk_threads_enter();
    gtk_init(&argc, &argv);

}
