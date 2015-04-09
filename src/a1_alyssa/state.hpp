#ifndef STATE_HPP
#define STATE_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <lcm/lcm-cpp.hpp>

// core api
#include "vx/vx.h"
#include "vx/vx_util.h"
#include "vx/vx_remote_display_source.h"
#include "vx/gtk/vx_gtk_display_source.h"

// drawables
#include "vx/vxo_drawables.h"

// common
#include "common/getopt.h"
#include "common/pg.h"
#include "common/zarray.h"

// imagesource
#include "imagesource/image_u32.h"
#include "imagesource/image_source.h"
#include "imagesource/image_convert.h"

#include "eecs467_util.h"    // This is where a lot of the internals live

#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"
#include "lcmtypes/maebot_pose_t.hpp"
#include "lcmtypes/maebot_motor_feedback_t.hpp"

#include <vector>
#include "OccupancyGridMapper.hpp"
#include "Slam.hpp"

struct state_t
{
    int running;

    double joy_bounds;
    double last_click[3];

    maebot_motor_command_t cmd;
    pthread_mutex_t cmd_mutex;
    pthread_t cmd_thread;

    //pthread_t lcm_thread;
    pthread_t render_thread;
    pthread_mutex_t render_mutex;
    pthread_mutex_t layer_mutex;

    pthread_t update_map_thread;

    getopt_t *gopt;
    char *url;
    image_source_t *isrc;
    int fidx;

    lcm::LCM *lcm;
    pthread_mutex_t lcm_mutex;

    OccupancyGridMapper *grid_mapper;
    Slam *slam;
    // bot info
    bool need_init;
    bool has_feedback;
    double right_prev_dist;
    double left_prev_dist;
    double next_x;
    double next_y;

    location loc;
    maebot_motor_feedback_t feedback; 
};

struct gui_state {
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;
    pthread_mutex_t gui_mutex;

    // threads stuff
    pthread_t animate_thread;
    pthread_t lcm_thread;

    // LCM stuff
    lcm::LCM *lcm;
    pthread_mutex_t lcm_mutex;

    // for accessing the arrays stuff
    pthread_mutex_t mutex;

    // occupancy grid stuff
    eecs467::OccupancyGrid grid;
    std::vector<maebot_pose_t> poses;
    std::vector<maebot_pose_t> particles;
    //std::vector<maebot_pose_t> truePoses;
    std::vector<maebot_pose_t> endpoints;
    std::vector<maebot_pose_t> pathPoints;

    int location_count;
};

struct error_state {
    bool running;

    getopt_t        *gopt;
    parameter_gui_t *pg;

    // image stuff
    char *img_url;
    int   img_height;
    int   img_width;

    // vx stuff
    vx_application_t    vxapp;
    vx_world_t         *vxworld;      // where vx objects are live
    vx_event_handler_t *vxeh; // for getting mouse, key, and touch events
    vx_mouse_event_t    last_mouse_event;
    pthread_mutex_t gui_mutex;

    // threads stuff
    pthread_t animate_thread;
    pthread_t lcm_thread;

    // LCM stuff
    lcm::LCM *lcm;
    pthread_mutex_t lcm_mutex;

    // for accessing the arrays stuff
    pthread_mutex_t mutex;

    // occupancy grid stuff
    eecs467::OccupancyGrid grid;
    std::vector<maebot_pose_t> poses;
    std::vector<maebot_pose_t> truePoses;
};

#endif
