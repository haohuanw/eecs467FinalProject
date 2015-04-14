#ifndef MAEBOT_GUI_T
#define MAEBOT_GUI_T
#include <deque>
#include "math/point.hpp"
#include "hsv.hpp"

#define NUM_MAEBOT 1

enum maebot_color {RED,BLUE,GREEN,NONE};

struct maebot_gui_t{
    maebot_color color;
    eecs467::Point<double> curr_pos;
    eecs467::Point<double> particle_pos;
    max_min_hsv hsv_range;
    std::deque<eecs467::Point<double>> path;
    std::deque<eecs467::Point<double>> waypoints; 
    eecs467::Point<double> curr_dest;
    uint32_t disp_color;
};

#endif
