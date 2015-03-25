#include "vx_state_t.hpp"

vx_state_t::vx_state_t(std::string name){
    click_point.x = -1;
    click_point.y = -1;
    vxname = name;
    vxworld = vx_world_create();
    printf("%s : create world\n",vxname.c_str());
    vxeh = (vx_event_handler_t*) calloc(1,sizeof(*vxeh)); 
    vxeh->mouse_event = vx_state_t::mouse_event;
    vxeh->touch_event = vx_state_t::touch_event;
    vxeh->key_event = vx_state_t::key_event;
    vxeh->dispatch_order = 100;
    vxeh->impl = this;
    printf("%s : create vxeh\n",vxname.c_str());
    vxapp.impl = this;
    vxapp.display_started = vx_state_t::display_started;
    vxapp.display_finished = vx_state_t::display_finished;
    layers = zhash_create(sizeof(vx_display_t*),sizeof(vx_layer_t*),zhash_ptr_hash,zhash_ptr_equals);
    pthread_mutex_init (&mutex, NULL);
}

vx_state_t::vx_state_t():vx_state_t("default window"){}

vx_state_t::~vx_state_t(){
    vx_world_destroy(vxworld);
    if(zhash_size(layers) != 0){
        zhash_destroy(layers);
    }
    pthread_mutex_destroy(&mutex);
}

eecs467::Point<float> vx_state_t::get_click_point(){
    eecs467::Point<float> tmp;
    tmp.x = click_point.x;
    tmp.y = click_point.y;
    click_point.x = -1;
    click_point.y = -1;
    return tmp;
}



