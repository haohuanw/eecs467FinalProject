#include "a2_intel.hpp"

INTEL::INTEL() { }

eecs467::Point<int> INTEL::nextMove(const BoardState& boardstate, const char& playerID) {

	return AI(boardstate, playerID);

}

eecs467::Point<int> INTEL::AI(const BoardState& boardstate, const char& playerID) {

	eecs467::Point<int> nextMove;

	//boardstate.displayBoard();

	//simple AI for now, put ball in the next available cell
	//going from position 0 to position 8
	for (unsigned int  i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {	
			if (boardstate.board[i][j] == '.') {
				nextMove.x = i;
				nextMove.y = j;		
			}
		}
	}	

	std::cout << playerID << " moves to position ( " << nextMove.x << "," << 
	nextMove.y << ")" << std::endl;
	
	return nextMove;
}

bool INTEL::Draw(const BoardState& boardstate, const char& playerID) {

	for (unsigned int  i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {		
			if (boardstate.board[i][j] != 'R' && boardstate.board[i][j] != 'G')
				return false;	
		}
	}
	
	std::cout << "It is a draw!" << std::endl;
	return true;
}

bool INTEL::Win(const BoardState& boardstate, const char& playerID) {

	char winID = WinOrLose(boardstate, playerID);

	if (winID == playerID)
		return true;
	else 
		return false;
}

bool INTEL::Lose(const BoardState& boardstate, const char& playerID) {

	char winID = WinOrLose(boardstate, playerID);

	if (winID == playerID)
		return false;
	if (winID == 'N')
		return false;
	else 
		return true;
}

char INTEL::WinOrLose(const BoardState& boardstate, const char& playerID) {

	char winner = 'N'; //none
	
	for (unsigned int i = 0; i < 3; i++) {

		//across
		if ( boardstate.board[i][0] == boardstate.board[i][1] && boardstate.board[i][1] == boardstate.board[i][2] )
			winner = boardstate.board[i][0];
	
		//down
		else if ( boardstate.board[0][i] == boardstate.board[1][i] && boardstate.board[1][i] == boardstate.board[2][i] ) 
			winner = boardstate.board[0][i];
	
		//forward diagnol	
		else if ( boardstate.board[0][0] == boardstate.board[1][1] && boardstate.board[1][1] == boardstate.board[2][2] ) 
			winner = boardstate.board[0][0];
	
		//backward diagnol	
		else if ( boardstate.board[0][2] == boardstate.board[1][1] && boardstate.board[1][1] == boardstate.board[2][0] ) 
			winner = boardstate.board[0][2];
	}
	
	return winner;
}

