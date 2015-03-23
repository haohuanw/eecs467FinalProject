#ifndef INTEL_HPP
#define INTEL_HPP

#include <vector>
#include <iostream>

//A2
#include "board_state.hpp"
#include "game_constants.hpp"

#include "math/point.hpp"

class INTEL {

private:  	

	char WinOrLose(const BoardState& boardstate, const char& playerID);
	
	/*
		chooses next position to place ball
		PlayerID is defined in game_constants.hpp
	*/
	eecs467::Point<int> AI(const BoardState& boardstate, const char& playerID);

public:

	ITNEL();

	bool Win(const BoardState& boardstate, const char& playerID);

	bool Lose(const BoardState& boardstate, const char& playerID);

	bool Draw(const BoardState& boardstate, const char& playerID);

	/*
		returns board coordinate where to put next ball
		before calling this function check if you have won, lost, or draw
	*/
	eecs467::Point<int> nextMove(const BoardState& boardstate, const char& playerID);

};

#endif
