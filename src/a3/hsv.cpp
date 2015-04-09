#include "hsv.hpp"
#include <cfloat>
#include <stdio.h>
#include <stdlib.h>

max_min_hsv::max_min_hsv(){
    max_HSV.H = FLT_MIN;
    max_HSV.S = FLT_MIN;
    max_HSV.V = FLT_MIN;
    min_HSV.H = FLT_MAX;
    min_HSV.S = FLT_MAX;
    min_HSV.V = FLT_MAX;
}

void max_min_hsv::updateHSV(hsv_color_t new_HSV){
    if(new_HSV.H > max_HSV.H){
        max_HSV.H = new_HSV.H;
    }
    if(new_HSV.H < min_HSV.H){
        min_HSV.H = new_HSV.H;
    }
    if(new_HSV.V > max_HSV.V){
        max_HSV.V = new_HSV.V;
    }
    if(new_HSV.V < min_HSV.V){
        min_HSV.V = new_HSV.V;
    }
    if(new_HSV.S > max_HSV.S){
        max_HSV.S = new_HSV.S;
    }
    if(new_HSV.S < min_HSV.S){
        min_HSV.S = new_HSV.S;
    }
}

void max_min_hsv::read_hsv_from_file(char const *filename){
    FILE *fp = fopen(filename,"r");
    if(fp == NULL){
        printf("Can't open file %s, ABORT.\n",filename);
        exit(EXIT_FAILURE);
    }
    else{
        fscanf(fp,"%f %f %f %f %f %f\n",&min_HSV.H,&max_HSV.H,&min_HSV.S,&max_HSV.S,&min_HSV.V,&max_HSV.V);
    }
}

hsv_color_t max_min_hsv::get_max_HSV(){
    return max_HSV;
}

hsv_color_t max_min_hsv::get_min_HSV(){
    return min_HSV;
}

