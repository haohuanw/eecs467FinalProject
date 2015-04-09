#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include "math/matd.h"
#include <vector>
#include <fstream>
#include <iostream>
#include "hsv.hpp"
#include "math/point.hpp"

class calibration_t{
    private:
        matd_t *image_to_world_mat;
        matd_t *og_to_world_mat;
        matd_t *world_to_image_mat;
        matd_t *world_to_og_mat;
        max_min_hsv cyan_range;
        max_min_hsv green_range;
        max_min_hsv red_range;
        std::vector<float> mask;
    public:
        calibration_t();
        ~calibration_t();
        void read_image_to_world_mat(char const* filename);
        void read_og_to_world_mat(char const* filename);
        void read_world_to_image_mat(char const* filename);
        void read_world_to_og_mat(char const* filename);
        void read_cyan_range(char const* filename);
        void read_green_range(char const* filename);
        void read_red_range(char const* filename);
        void read_mask(char const* filename);
        max_min_hsv get_cyan();
        max_min_hsv get_green();
        max_min_hsv get_red();

        // [x1, x2, y1, y2]
        std::vector<float> get_mask();

        // [x, y]
        eecs467::Point<double> image_to_world_translate(eecs467::Point<double> im_coords);
        eecs467::Point<double> og_to_world_translate(eecs467::Point<double> og_coords);
        eecs467::Point<double> world_to_image_translate(eecs467::Point<double> world_coords);
        eecs467::Point<double> world_to_og_translate(eecs467::Point<double> world_coords);
};

#endif //CALIBRATION_HPP
