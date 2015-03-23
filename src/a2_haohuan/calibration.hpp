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
  matd_t *tx_mat;
  max_min_hsv cyan_range;
  max_min_hsv green_range;
  max_min_hsv red_range;
  std::vector<float> mask;
public:
  calibration_t();
  ~calibration_t();
  void read_tx_mat(char* filename);
  void read_cyan_range(char* filename);
  void read_green_range(char* filename);
  void read_red_range(char* filename);
  void read_mask(char* filename);
  void read_default_all();
  max_min_hsv get_cyan();
  max_min_hsv get_green();
  max_min_hsv get_red();

  // [x1, x2, y1, y2]
  std::vector<float> get_mask();

  // [x, y]
  eecs467::Point<double> translate(eecs467::Point<double> cam_coords);
};

#endif //CALIBRATION_HPP
