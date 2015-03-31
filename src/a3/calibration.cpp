#include "calibration.hpp"
#include <vector>
#include <fstream>
#include <iostream>
#include "hsv.hpp"
#include "math/matd.h"
#include "math/point.hpp"

calibration_t::calibration_t(){
    image_to_world_mat = matd_create(3,3);
    og_to_world_mat = matd_create(3,3);
    world_to_image_mat = matd_create(3,3);
    world_to_og_mat = matd_create(3,3);
    std::cout << "Constructor called" << std::endl;
}

calibration_t::~calibration_t(){
    matd_destroy(image_to_world_mat);
    matd_destroy(og_to_world_mat); 
    matd_destroy(world_to_image_mat); 
    matd_destroy(world_to_og_mat);     
    std::cout << "Destructor called\n";
}

void calibration_t::read_image_to_world_mat(char const* filename){
    std::ifstream f(filename);
    double e[6];
    if(f != NULL){
        f >> e[0] >> e[1] >> e[2] >> e[3] >> e[4] >> e[5];
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
        matd_put(image_to_world_mat, 0, 0, e[0]);
        matd_put(image_to_world_mat, 0, 1, e[1]);
        matd_put(image_to_world_mat, 0, 2, e[2]);
        matd_put(image_to_world_mat, 1, 0, e[3]);
        matd_put(image_to_world_mat, 1, 1, e[4]);
        matd_put(image_to_world_mat, 1, 2, e[5]);
        matd_put(image_to_world_mat, 2, 0, 0);
        matd_put(image_to_world_mat, 2, 1, 0);
        matd_put(image_to_world_mat, 2, 2, 1.0); 
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
    }
    else{
        printf("Error on opening matrix file\n");
        exit(1);
    }

}

void calibration_t::read_og_to_world_mat(char const* filename){
    std::ifstream f(filename);
    double e[6];
    if(f != NULL){
        f >> e[0] >> e[1] >> e[2] >> e[3] >> e[4] >> e[5];
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
        matd_put(og_to_world_mat, 0, 0, e[0]);
        matd_put(og_to_world_mat, 0, 1, e[1]);
        matd_put(og_to_world_mat, 0, 2, e[2]);
        matd_put(og_to_world_mat, 1, 0, e[3]);
        matd_put(og_to_world_mat, 1, 1, e[4]);
        matd_put(og_to_world_mat, 1, 2, e[5]);
        matd_put(og_to_world_mat, 2, 0, 0);
        matd_put(og_to_world_mat, 2, 1, 0);
        matd_put(og_to_world_mat, 2, 2, 1.0); 
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
    }
    else{
        printf("Error on opening matrix file\n");
        exit(1);
    }

}

void calibration_t::read_world_to_image_mat(char const* filename){
    std::ifstream f(filename);
    double e[6];
    if(f != NULL){
        f >> e[0] >> e[1] >> e[2] >> e[3] >> e[4] >> e[5];
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
        matd_put(world_to_image_mat, 0, 0, e[0]);
        matd_put(world_to_image_mat, 0, 1, e[1]);
        matd_put(world_to_image_mat, 0, 2, e[2]);
        matd_put(world_to_image_mat, 1, 0, e[3]);
        matd_put(world_to_image_mat, 1, 1, e[4]);
        matd_put(world_to_image_mat, 1, 2, e[5]);
        matd_put(world_to_image_mat, 2, 0, 0);
        matd_put(world_to_image_mat, 2, 1, 0);
        matd_put(world_to_image_mat, 2, 2, 1.0); 
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
    }
    else{
        printf("Error on opening matrix file\n");
        exit(1);
    }

}

void calibration_t::read_world_to_og_mat(char const* filename){
    std::ifstream f(filename);
    double e[6];
    if(f != NULL){
        f >> e[0] >> e[1] >> e[2] >> e[3] >> e[4] >> e[5];
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
        matd_put(world_to_og_mat, 0, 0, e[0]);
        matd_put(world_to_og_mat, 0, 1, e[1]);
        matd_put(world_to_og_mat, 0, 2, e[2]);
        matd_put(world_to_og_mat, 1, 0, e[3]);
        matd_put(world_to_og_mat, 1, 1, e[4]);
        matd_put(world_to_og_mat, 1, 2, e[5]);
        matd_put(world_to_og_mat, 2, 0, 0);
        matd_put(world_to_og_mat, 2, 1, 0);
        matd_put(world_to_og_mat, 2, 2, 1.0); 
        std::cout << e[0] << ' ' << e[1] << ' ' << e[2] << std::endl;
    }
    else{
        printf("Error on opening matrix file\n");
        exit(1);
    }

}
void calibration_t::read_red_range(char const* filename){
    red_range.read_hsv_from_file(filename);
}

void calibration_t::read_cyan_range(char const*filename){
    cyan_range.read_hsv_from_file(filename);
}

void calibration_t::read_green_range(char const*filename){
    green_range.read_hsv_from_file(filename);
}

void calibration_t::read_mask(char const* filename){
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

eecs467::Point<double> calibration_t::image_to_world_translate(eecs467::Point<double> im_coords){
    eecs467::Point<double> world_coords;
    world_coords.x = 
        matd_get(image_to_world_mat, 0, 0)*im_coords.x
      + matd_get(image_to_world_mat, 0, 1)*im_coords.y 
      + matd_get(image_to_world_mat, 0, 2)*1.0;
    world_coords.y = 
        matd_get(image_to_world_mat, 1, 0)*im_coords.x 
      + matd_get(image_to_world_mat, 1, 1)*im_coords.y 
      + matd_get(image_to_world_mat, 1, 2)*1.0;
    return world_coords; 
}

eecs467::Point<double> calibration_t::og_to_world_translate(eecs467::Point<double> og_coords){
    eecs467::Point<double> world_coords;
    world_coords.x = 
        matd_get(og_to_world_mat, 0, 0)*og_coords.x
      + matd_get(og_to_world_mat, 0, 1)*og_coords.y 
      + matd_get(og_to_world_mat, 0, 2)*1.0;
    world_coords.y = 
        matd_get(og_to_world_mat, 1, 0)*og_coords.x 
      + matd_get(og_to_world_mat, 1, 1)*og_coords.y 
      + matd_get(og_to_world_mat, 1, 2)*1.0;
    return world_coords; 
}


eecs467::Point<double> calibration_t::world_to_image_translate(eecs467::Point<double> world_coords){
    eecs467::Point<double> image_coords;
    image_coords.x = 
        matd_get(world_to_image_mat, 0, 0)*world_coords.x
      + matd_get(world_to_image_mat, 0, 1)*world_coords.y 
      + matd_get(world_to_image_mat, 0, 2)*1.0;
    image_coords.y = 
        matd_get(world_to_image_mat, 1, 0)*world_coords.x 
      + matd_get(world_to_image_mat, 1, 1)*world_coords.y 
      + matd_get(world_to_image_mat, 1, 2)*1.0;
    return image_coords; 
}

eecs467::Point<double> calibration_t::world_to_og_translate(eecs467::Point<double> world_coords){
    eecs467::Point<double> og_coords;
    og_coords.x = 
        matd_get(world_to_og_mat, 0, 0)*world_coords.x
      + matd_get(world_to_og_mat, 0, 1)*world_coords.y 
      + matd_get(world_to_og_mat, 0, 2)*1.0;
    og_coords.y = 
        matd_get(world_to_og_mat, 1, 0)*world_coords.x 
      + matd_get(world_to_og_mat, 1, 1)*world_coords.y 
      + matd_get(world_to_og_mat, 1, 2)*1.0;
    return og_coords; 
}


