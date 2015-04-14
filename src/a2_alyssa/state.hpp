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

#include <vector>

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
