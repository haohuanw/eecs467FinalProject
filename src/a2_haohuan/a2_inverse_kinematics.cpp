#include "a2_inv_kin.hpp"
using namespace std;

typedef struct state state_t;
struct state
{
  getopt_t *gopt;

  // LCM
  lcm_t *lcm;
  const char *command_channel;
  const char *status_channel;
  
  thread status_thread;
  thread command_thread;
};

state_t state;

static void
status_handler (const lcm_recv_buf_t *rbuf,
                const char *channel,
                const dynamixel_status_list_t *msg,
                void *user)
{
}

void
status_loop ()
{
  
  dynamixel_status_list_t_subscribe (state.lcm,
				     state.status_channel,
				     status_handler,
				     &state);
  const int hz = 15;
  while (1) {
    // Set up the LCM file descriptor for waiting. This lets us monitor it
    // until something is "ready" to happen. In this case, we are ready to
    // receive a message.
    int status = lcm_handle_timeout (state.lcm, 1000/hz);
    if (status <= 0)
      continue;

    // LCM has events ready to be processed
  }

  return;
}

void move_to_position(vector<double> coords, dynamixel_command_list_t cmds){
  /* coords:
   * [0] = x
   * [1] = y
   * [2] = z
   * [3] = wrist bend
   * [4] = wrist_rotate
   */
  vector<double> angles(5);
  /* angle[0] = base
   * angle[1] = shoulder
   * angle[2] = elbow
   * angle[3] = wrist bend
   * angle[4] = wrist rotate
   */
  
  double h = coords[2];
  double R_squared = sq(coords[0]) + sq(coords[1]);
  double M_squared = R_squared + sq(d4 + h - d1);
  double alpha = atan2(d4 + h - d1, sqrt(R_squared));
  double beta = facos((-(d3*d3) + (d2*d2) + M_squared) / (2.0 * d2 * sqrt(M_squared)));
  double gamma = facos(((-1)*M_squared + (d2*d2) + (d3*d3)) / (2 * d2 * d3));

  angles[0] = atan2(coords[1], coords[0]);
  angles[1] = (M_PI/2.0) - alpha - beta;
  angles[2] = M_PI - gamma;
  angles[3] = M_PI - angles[1] - angles[2];
  angles[4] = 0;


  //cout << "R_squared: " << R_squared << endl;
  //cout << "M_squared: " << M_squared << endl;
  //cout << "alpha: " << alpha << endl;
  //cout << "beta_num: " << (-(d3*d3) + (d2*d2) + M_squared) << endl;
  //cout << "beta_denom: " << (2.0 * d2 * sqrt(M_squared)) << endl;
  //cout << "beta: " << beta << endl << endl;
  //cout << "gamma_num: " << ((-1)*M_squared + (d2*d2) + (d3*d3)) << endl;
  //cout << "gamma_denom: " << (2 * d2 * d3) << endl;
  //cout << "gamma: " << gamma << endl << endl;
  //cout << endl;
  cout << "Base: " << angles[0] << endl;
  cout << "Shoulder: " << angles[1] << endl;
  cout << "Elbow: " << angles[2] << endl;
  cout << "Wrist Bend: " << angles[3] << endl;
  cout << "Wrist Rotate: " << angles[4] << endl;

  int servo_order[5] = {0, 4, 3, 2, 1};
  for( auto i : servo_order){
    //cout << "angles[" << i << "]: " << angles[i] << endl;
    //if(i == 0){
    cmds.commands[i].utime = utime_now();
    cmds.commands[i].position_radians = angles[i];
    dynamixel_command_list_t_publish (state.lcm, state.command_channel, &cmds);
    //}
    usleep (1000000);
  }
  cout << endl << endl;


  return;
}

void command_loop()
{
  inv_kinematics ik(state.lcm, state.command_channel);
  while(1){
    double x, y;
    cout << "Input an x,y pair" << endl;
    cin >> x >> y;
    cout << x << '\t' << y << endl;
    ik.move_2_pos(x, y);
    usleep(3000000);
    ik.go_home();
    usleep(3000000);
  }
}


int main(int argc, char *argv[])
{

  getopt_t *gopt = getopt_create ();
  getopt_add_bool (gopt, 'h', "help", 0, "Show this help screen");
  getopt_add_bool (gopt, 'i', "idle", 0, "Command all servos to idle");
  getopt_add_string (gopt, '\0', "status-channel", "ARM_STATUS", "LCM status channel");
  getopt_add_string (gopt, '\0', "command-channel", "ARM_COMMAND", "LCM command channel");

  if (!getopt_parse (gopt, argc, argv, 1) || getopt_get_bool (gopt, "help")) {
    getopt_do_usage (gopt);
    exit (EXIT_FAILURE);
  }
  
  state.gopt = gopt;
  state.lcm = lcm_create (NULL);
  state.command_channel = getopt_get_string (gopt, "command-channel");
  state.status_channel = getopt_get_string (gopt, "status-channel");

  state.status_thread = thread(status_loop);
  state.command_thread = thread(command_loop);

  state.status_thread.join();
  state.command_thread.join();

  lcm_destroy(state.lcm);
  getopt_destroy(gopt);
  return 0;
}
