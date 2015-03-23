#include "gameboard.hpp"
#include <stdio.h>
#include <cstdlib>
#include <vector>
#include "arm_ai.hpp"
gameboard::gameboard(){
    for(int i=0;i<9;++i){
        //board[i] = ' ';
        board.push_back(' ');
    }
}

void gameboard::print_board(){
    printf(" %c | %c | %c\n",board[0],board[1],board[2]);
    printf("---+---+---\n");
    printf(" %c | %c | %c\n",board[3],board[4],board[5]);
    printf("---+---+---\n");
    printf(" %c | %c | %c\n",board[6],board[7],board[8]);
}

void gameboard::update_board(int move, char pieces){
    if(move >=0 && move < 9 && board[move] == ' '){
        board[move] = pieces;
    }
}

void gameboard::update_entire_board(std::vector <char> b){
	for(int i=0;i<9;++i){
		board[i] = b[i];
	}
}

int gameboard::is_win(char pieces){
    arm_ai ai = arm_ai(pieces);
    return ai.is_win(this->get_board(pieces));
}

bool gameboard::is_valid(char piece1, char piece2){
    int p1_count = 0;//1
    int p2_count = 0;//-1
    for(int i=0;i<9;++i){
        if(board[i] == piece1){
            ++p1_count;
        }
        else if(board[i] == piece2){
            ++p2_count;
        }
        else if(board[i] == ' '){
            continue;
        }
        else{
            return false;
        }
    }
    if(abs(p1_count - p2_count) > 1){
        return false;
    }
    return true;
}

bool gameboard::is_finished(){
    for(int i=0;i<9;++i){
        if(board[i] == ' '){
            return false;
        }
    }
    return true;
}

std::vector<int> gameboard::get_board(char pieces){
    std::vector<int> b;
    for(int i=0;i<9;++i){
        if(board[i] == pieces){
            //b[i] = 1;
            b.push_back(1);
        }
        else if(board[i] == ' '){
            //b[i] = 0;
            b.push_back(0);
        }
        else{
            //b[i] = -1;
            b.push_back(-1);
        }
        //printf("%d ",b[i]);
    }
    //printf("\n");
    return b;
}

bool gameboard::operator==(const gameboard& rhs){
  for(int i=0;i<9;++i){
    if(this->board[i] != rhs.board[i]){
      return false;
    }
  }
  return true;
}
