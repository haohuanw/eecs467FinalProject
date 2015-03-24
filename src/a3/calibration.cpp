#include "calibration.hpp"
#include <vector>
#include <fstream>
#include <iostream>
#include "hsv.hpp"
#include "math/matd.h"
#include "math/point.hpp"

calibration_t::calibration_t(){
  tx_mat = matd_create(3,3);
  std::cout << "Constructor called" << std::endl;
}

calibration_t::~calibration_t(){
  matd_destroy(tx_mat);
  std::cout << "Destructor called\n";
}


void calibration_t::read_tx_mat(char* filename){
  std::ifstream f(filename);
  double e[6];
  if(f != NULL){
    f >> e[0] >> e[1] >> e[2] >> e[3] >> e[4] >> e[5];
    std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
    matd_put(tx_mat, 0, 0, e[0]);
    matd_put(tx_mat, 0, 1, e[1]);
    matd_put(tx_mat, 0, 2, e[2]);
    matd_put(tx_mat, 1, 0, e[3]);
    matd_put(tx_mat, 1, 1, e[4]);
    matd_put(tx_mat, 1, 2, e[5]);
    matd_put(tx_mat, 2, 0, 0);
    matd_put(tx_mat, 2, 1, 0);
    matd_put(tx_mat, 2, 2, 1.0); 
    std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
  }
  else{
    printf("Error on opening matrix file\n");
    exit(1);
  }
    
}

void calibration_t::read_red_range(char* filename){
    red_range.read_hsv_from_file(filename);
}

void calibration_t::read_cyan_range(char *filename){
    cyan_range.read_hsv_from_file(filename);
}

void calibration_t::read_green_range(char *filename){
    green_range.read_hsv_from_file(filename);
}

void calibration_t::read_mask(char* filename){
    std::ifstream f(filename); 
    float e[4];
    if(f != NULL){
      f >> e[0] >> e[1] >> e[2] >> e[3];
      mask.push_back(e[0]);
      mask.push_back(e[1]);
      mask.push_back(e[2]);
      mask.push_back(e[3]);
    }
    else{
        printf("Error on opening mask file\n");
        exit(1);
    } 
}

max_min_hsv calibration_t::get_cyan(){
    return cyan_range;
}

max_min_hsv calibration_t::get_green(){
    return green_range;
}

max_min_hsv calibration_t::get_red(){
    return red_range;
}

std::vector<float> calibration_t::get_mask(){
    return mask;
}

eecs467::Point<double> calibration_t::translate(eecs467::Point<double> cam_coords){
    eecs467::Point<double> ground_in_world;
    ground_in_world.x = 
            matd_get(tx_mat, 0, 0)*cam_coords.x
            + matd_get(tx_mat, 0, 1)*cam_coords.y 
            + matd_get(tx_mat, 0, 2)*1.0;
    ground_in_world.y = 
            matd_get(tx_mat, 1, 0)*cam_coords.x 
            + matd_get(tx_mat, 1, 1)*cam_coords.y 
            + matd_get(tx_mat, 1, 2)*1.0;
    return ground_in_world; 
}

