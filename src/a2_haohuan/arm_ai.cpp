#include "arm_ai.hpp"
#include <stdio.h>
#include <vector>
arm_ai::arm_ai(char p){ 
    player = p;
}

char arm_ai::get_player(){
    return player;
}

int arm_ai::min_max(std::vector<int> &board,int player){ 
    //How is the position like for player (their turn) on board?
    int winner = is_win(board);
    if(winner != 0) return winner*player;
    int move = -1;
    int score = -2;//Losing moves are preferred to no move
    for(int i = 0; i < 9; ++i) {//For all moves,
        if(board[i] == 0) {//If legal,
            board[i] = player;//Try the move
            int thisScore = -min_max(board, player*-1);
            if(thisScore > score) {
                score = thisScore;
                move = i;
            }//Pick the one that's worst for the opponent
            board[i] = 0;//Reset board after try
        }
    }
    if(move == -1) return 0;
    return score;
}

int arm_ai::is_win(const std::vector<int> board) {
    //determines if a player has won, returns 0 otherwise.
    unsigned wins[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}};
    for(int i = 0; i < 8; ++i) {
        if(board[wins[i][0]] != 0 &&
                board[wins[i][0]] == board[wins[i][1]] &&
                board[wins[i][0]] == board[wins[i][2]])
            return board[wins[i][2]];
    }
    return 0;
}

int arm_ai::calc_move(std::vector<int> board){
    //for(int i=0;i<9;++i){
    //    printf("%d ",board[i]);
    //}
    //printf("\n");
    int move = -1;
    int score = -2;
    for(int i = 0; i < 9; ++i) {
        if(board[i] == 0) {
            board[i] = 1;
            int tempScore = -min_max(board, -1);
            board[i] = 0;
            if(tempScore > score) {
                score = tempScore;
                move = i;
            }
        }
    }
    //returns a score based on minimax tree at a given node.
    //board[move] = 1;
    return move; 
}
