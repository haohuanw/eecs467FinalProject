#ifndef BOARD_STATE_HPP
#define BOARD_STATE_HPP

#include <vector>
#include <iostream>
#include <algorithm>
//#include <queue>

//A2
#include "game_constants.hpp"
#include "calibration.hpp"

#include <cmath>
#include "math/math_util.h"
#include "math/point.hpp"

class BoardState {
	
public:
	
  std::vector < eecs467::Point <double> > availBalls; 
  std::vector<eecs467::Point<int>> cyan_squares;
  std::vector<eecs467::Point<double>> board_squares;

public:

  BoardState();

  /*
    expects coordinates of the red and green balls in the img coordinate frame
    and determines the state of the board	
  */	
  std::vector <char> determineStateofBoard(std::vector <int>& greenBalls, 
					   std::vector <int>& redBalls, 
					   int imgWidth, 
					   int imgHeight,
					   calibration_t &cali, 
					   char color);	

  // returns num of balls left to place
  int ballsLeft();
	
  eecs467::Point <double> nextFreeBall();

  int convert(eecs467::Point<double> p, eecs467::Point<double> sq1, eecs467::Point<double> sq4);
	
};

#endif
