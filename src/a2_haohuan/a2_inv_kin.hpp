#ifndef INV_KIN_HPP
#define INV_KIN_HPP

#include <iostream>
#include <fstream>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <sys/select.h>
#include <sys/time.h>
#include <thread>
#include <vector>

#include <lcm/lcm.h>
#include "lcmtypes/dynamixel_command_list_t.h"
#include "lcmtypes/dynamixel_command_t.h"
#include "lcmtypes/dynamixel_status_list_t.h"
#include "lcmtypes/dynamixel_status_t.h"

#include "common/getopt.h"
#include "common/timestamp.h"
#include "math/math_util.h"
#include "math/matd.h"
#include "math/fasttrig.h"
using namespace std;

#define NUM_SERVOS 6

const double inches_to_meters = 0.0254;

const double d1 = 0.12;
const double d2 = 0.10;
const double d3 = 0.10;
const double d4 = 0.115;

const double h = 0.089;

const double FINGERS_CLOSE = 1.6;
const double FINGERS_OPEN_PICKUP = 1.25;
const double FINGERS_OPEN_PLACE = 1.4;

class inv_kinematics{
public:
  inv_kinematics();
  inv_kinematics(lcm_t *l, const char *com_chan);
  ~inv_kinematics();
  void move_2_pos(double x, double y);
  void go_home();
  void pick_up(double x, double y);
  void place(double x, double y);
  void place_08(int pos);
  void wave();
  void set_cmds(dynamixel_command_list_t &command_list);
  void set_lcm(lcm_t *l);
  void set_com_channel(const char *com_chan);
private:
  dynamixel_command_list_t cmds;
  lcm_t* lcm;
  char *command_channel;
};

#endif //INV_KIN_HPP
