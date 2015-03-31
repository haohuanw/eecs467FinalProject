#ifndef MAEBOT_GUI_T
#define MAEBOT_GUI_T
#include <deque>
#include "math/point.hpp"
#include "hsv.hpp"
enum maebot_color {RED,BLUE,NONE};

struct maebot_gui_t{
    maebot_color color;
    eecs467::Point<double> curr_pos;
    max_min_hsv hsv_range;
    std::deque<eecs467::Point<double>> path;
    std::deque<eecs467::Point<double>> waypoints; 
    uint32_t disp_color;
};

#endif
