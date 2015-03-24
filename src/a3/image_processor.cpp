#include "image_processor.hpp"
#include <algorithm>
#include <cmath>
#include <climits>
#include <cfloat>
#include <deque>
#define BLACK 0xff000000
void image_processor::image_masking(image_u32_t *im,float x1,float x2,float y1, float y2){
    uint32_t black = 0xff000000;
    for (int y = 0; y < im->height; ++y) {
        for (int x = 0; x < im->width; ++x) {
            if(x<x1 || x>x2 || y<(im->height-y2) || y>(im->height-y1)){
                im->buf[y*im->stride + x] = black;
            }
        }
    }  
}

std::vector<int> image_processor::blob_detection(image_u32_t *im, float x1,float x2,float y1,float y2,max_min_hsv color){
    uint32_t black = 0xff000000;
    hsv_color_t tmp_hsv;
    hsv_color_t max_hsv = color.get_max_HSV();
    hsv_color_t min_hsv = color.get_min_HSV();
    int real_y1 = (int)(im->height - y2);
    int real_y2 = (int)(im->height - y1);
    int real_height = real_y2-real_y1+1;
    int real_width = x2-x1+1;
    //initialize with all 0 => black
    int im_section[im->height * im->width];
    std::deque<int> not_black_pos;
    std::vector<std::vector<int>> section_list;
    std::vector<int> center_list;
    //black out pixels that is not in the hsv range
    for(int y = real_y1;y<=real_y2;++y){
        for(int x = (int)x1;x<=(int)x2;++x){
            tmp_hsv = rgb_to_hsv(im->buf[y*im->stride + x]);    
            if(is_hsv_in_range(tmp_hsv,max_hsv,min_hsv)){
                not_black_pos.push_back(y*im->width+x);
                //not grouped
                im_section[y*im->width+x] = 1;
            }
        }
    }
    while(!not_black_pos.empty()){
        int not_black_curr_pos = not_black_pos.back();
        not_black_pos.pop_back();
        //if this is not already grouped with others
        if(im_section[not_black_curr_pos] == 1){
            std::deque<int> im_selection_pos;
            std::vector<int> section;
            //deselected
            im_selection_pos.push_back(not_black_curr_pos);
            section.push_back(not_black_curr_pos);
            while(!im_selection_pos.empty()){
                int selection_curr_pos = im_selection_pos.back();
                im_selection_pos.pop_back();
                //mark as grouped
                im_section[selection_curr_pos] = 2; 
                for(int i=0;i<8;++i){
                    int new_pos;
                    switch (i){
                        case 0:
                            new_pos = selection_curr_pos - 1;
                            break;
                        case 1:
                            new_pos = selection_curr_pos + 1;
                            break;
                        case 2:
                            new_pos = selection_curr_pos - im->width;
                            break;
                        case 3:
                            new_pos = selection_curr_pos + im->width;
                            break;
                        case 4:
                            new_pos = selection_curr_pos - im->width - 1;
                            break;
                        case 5:
                            new_pos = selection_curr_pos - im->width + 1;
                            break;
                        case 6:
                            new_pos = selection_curr_pos + im->width - 1;
                            break;
                        case 7:
                            new_pos = selection_curr_pos + im->width + 1;
                            break; 
                    }
                    //if the surrounding is not black and not grouped by other group
                    if(im_section[new_pos] == 1){
                        //grouped by others
                        im_section[new_pos] = 2;
                        im_selection_pos.push_back(new_pos);
                        section.push_back(new_pos);
                    }
                }
            }//finish bucket fill in one section
            section_list.push_back(section);
        }
    }
    //delete small region and find the center of big region
    for(auto it = section_list.begin();it!=section_list.end();++it){
        if(it->size()>50){
            //this is a blob
            int y_max = INT_MIN;
            int y_min = INT_MAX;
            int x_max = INT_MIN;
            int x_min = INT_MAX;
            for(auto i = it->begin();i!=it->end();++i){
                int point_y = (*i)/im->width;
                int point_x = (*i)%im->width; 
                if(point_y > y_max){
                    y_max = point_y;
                }
                if(point_y < y_min){
                    y_min = point_y;
                }
                if(point_x > x_max){
                    x_max = point_x;
                }
                if(point_x < x_min){
                    x_min = point_x;
                }
            }
            float radius = ((x_max-x_min+1)+(y_max-y_min+1))/4.0;
            int center_x = (x_max+x_min)/2;
            int center_y = (y_max+y_min)/2;
            int center_pos = center_y*im->width + center_x;
            center_list.push_back(center_pos);
        }
    }
    return center_list;
}

void image_processor::draw_circle(image_u32_t *im,int center_x,int center_y, float radius,uint32_t color){
    float radius_2 = radius*radius;
    int y_min = center_y - radius;
    int y_max = center_y + radius;
    int x_min = center_x - radius;
    int x_max = center_x + radius;
    for(int y=y_min;y<=y_max;++y){
		for(int x=x_min;x<=x_max;++x){
		    int dis = (x-center_x)*(x-center_x) + (y-center_y)*(y-center_y);
			if(dis <= (radius_2)){
				im->buf[y*im->stride+x] = color;
			}	
		}
	}
}

void image_processor::image_select(image_u32_t *im,max_min_hsv hsv){
    hsv_color_t max_hsv = hsv.get_max_HSV();
    hsv_color_t min_hsv = hsv.get_min_HSV();
    hsv_color_t tmp_hsv;
    uint32_t light_purple = 0xffff73df;
    for (int y = 0; y < im->height; ++y) {
        for (int x = 0; x < im->width; ++x) {
            tmp_hsv = rgb_to_hsv(im->buf[y*im->stride + x]);    
            if(is_hsv_in_range(tmp_hsv,max_hsv,min_hsv)) {
                im->buf[y*im->stride + x] = light_purple; 
            }
        }
    }  
}


hsv_color_t image_processor::rgb_to_hsv(uint32_t abgr){
    hsv_color_t hsv_tmp;
    uint8_t r = (uint8_t)  abgr;
    uint8_t g = (uint8_t) (abgr >> 8);
    uint8_t b = (uint8_t) (abgr >> 16);
    float r_prime = r/255.0;
    float g_prime = g/255.0;
    float b_prime = b/255.0;
    float cmax = std::max(r_prime,std::max(g_prime,b_prime));
    float cmin = std::min(r_prime,std::min(g_prime,b_prime));
    float delta = cmax - cmin;
    hsv_tmp.V = cmax;
    if(fabs(cmax - 0.00) > FLT_EPSILON){
        hsv_tmp.S = delta/cmax;
    }
    else{
        //r = g = b = 0
        hsv_tmp.H = -1;
        hsv_tmp.S = 0;
    }
    if(fabs(cmax - r_prime) < FLT_EPSILON){
        hsv_tmp.H = (g_prime - b_prime) / delta; 
    }
    else if(fabs(cmax - g_prime) < FLT_EPSILON){
        hsv_tmp.H = (2 + b_prime - r_prime) / delta;
    }
    else{
        hsv_tmp.H = (4 + r_prime - g_prime) / delta;
    }
    hsv_tmp.H *= 60.0;
    if(hsv_tmp.H < 0){
        hsv_tmp.H += 360.0;
    }
    return hsv_tmp;
} 

bool image_processor::is_hsv_in_range(hsv_color_t hsv,hsv_color_t max,hsv_color_t min){
    return hsv.H <= max.H && hsv.S <= max.S && hsv.V <= max.V &&
        hsv.H >= min.H && hsv.S >= min.S && hsv.V >= min.V;
}
