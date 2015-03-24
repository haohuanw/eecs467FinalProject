#ifndef VX_STATE_T
#define VX_STATE_T

static void display_finished(vx_application_t * app, vx_display_t * disp);
static void display_started(vx_application_t * app, vx_display_t * disp);

class vx_state_t{
    public:
        vx_application_t vxapp;
        vx_world_t       *vxworld;
        zhash_t          *layers;
        vx_gtk_display_source_t* appwrap;
        pthread_mutex_t  mutex;
    public:
        vx_state_t(){
            vxworld = vx_world_create();
            printf("create world\n");
            vxapp.impl = this;
            vxapp.display_started = display_started;
            vxapp.display_finished = display_finished;
            layers = zhash_create(sizeof(vx_display_t*),sizeof(vx_layer_t*),zhash_ptr_hash,zhash_ptr_equals);
            pthread_mutex_init (&mutex, NULL);
        }
        ~vx_state_t(){
            vx_world_destroy(vxworld);
            if(zhash_size(layers) != 0){
                zhash_destroy(layers);
            }
            pthread_mutex_destroy(&mutex);
        }
};

static void display_finished(vx_application_t * app, vx_display_t * disp)
{
    vx_state_t * state = (vx_state_t*)app->impl;
    pthread_mutex_lock(&state->mutex);

    vx_layer_t * layer = NULL;

    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_remove(state->layers, &disp, NULL, &layer);

    vx_layer_destroy(layer);

    pthread_mutex_unlock(&state->mutex);
}

static void display_started(vx_application_t * app, vx_display_t * disp)
{
    vx_state_t * state = (vx_state_t*)app->impl;

    vx_layer_t * layer = vx_layer_create(state->vxworld);
    vx_layer_set_display(layer, disp);
    //important setting for the handler
    //if(state->vxeh != NULL){
    //    vx_layer_add_event_handler(layer,state->vxeh);
    //}
    pthread_mutex_lock(&state->mutex);
    // store a reference to the world and layer that we associate with each vx_display_t
    zhash_put(state->layers, &disp, &layer, NULL, NULL);
    pthread_mutex_unlock(&state->mutex);
}

#endif
