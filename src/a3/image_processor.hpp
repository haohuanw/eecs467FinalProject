#ifndef IMAGE_PROCESSOR_HPP
#define IMAGE_PROCESSOR_HPP
#include "imagesource/image_u32.h"
#include "hsv.hpp"
#include <vector>
class image_processor{
    public:
        image_processor(){}
        void image_masking(image_u32_t *im,float x1,float x2,float y1,float y2);
        std::vector<int> blob_detection(image_u32_t *im, float x1,float x2,float y1,float y2,max_min_hsv color);
	    void draw_circle(image_u32_t *im,int center_x,int center_y,float radius,uint32_t color);
        hsv_color_t rgb_to_hsv(uint32_t abgr); 
        bool is_hsv_in_range(hsv_color_t hsv,hsv_color_t max, hsv_color_t min);
        void image_select(image_u32_t *im,max_min_hsv hsv);
        void image_copy(image_u32_t *im,image_u32_t *im_copy_from,float x1,float x2,float y1,float y2);
};


#endif
