#ifndef HSV_HPP
#define HSV_HPP

struct hsv_color_t{
    float H;
    float S;
    float V;
};

class max_min_hsv{
    public:
        max_min_hsv();
        void updateHSV(hsv_color_t new_HSV);
        void read_hsv_from_file(char const *filename);
        hsv_color_t get_max_HSV();
        hsv_color_t get_min_HSV();
    private:
        hsv_color_t max_HSV;
        hsv_color_t min_HSV;      
};

#endif
