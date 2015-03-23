#include <stdio.h>
#include "gameboard.hpp"
#include "arm_ai.hpp"
#include <vector>

int main(){
    arm_ai a1 = arm_ai('R');
    arm_ai a2 = arm_ai('G');
    gameboard g = gameboard();
    while(1){
        printf("your turn\n");
        int move;
        scanf("%d",&move);
        printf("\n");
        g.update_board(move,'G');
        g.print_board();
        if(g.is_finished() || (g.is_win('G')!= 0)){
            break;
        }

        printf("player1 turn:\n");
        //std::vector<int> b = g.get_board(a1.get_player());
        //for(int i=0;i<9;++i){
        //    printf("%d ",b[i]);
        //}
        //printf("\n");
        //move = a1.calc_move(b);
        //printf("%d\n",move);
        //g.update_board(move,a1.get_player());
        g.update_board(a1.calc_move(g.get_board(a1.get_player())),a1.get_player());
        g.print_board();
        if(g.is_finished() || (g.is_win(a1.get_player())!= 0)){
            break;
        }
        //printf("your turn\n");
        //int move;
        //scanf("%d",&move);
        //printf("\n");
        //g.update_board(move,'G');

        //printf("player2 turn:\n");
        //g.update_board(a2.calc_move(g.get_board(a2.get_player())),a2.get_player());
        //g.print_board();
        //if(g.is_finished() || a1.is_win(g.get_board(a1.get_player()))){
        //    break;
        //}
    }
    switch(g.is_win('R')){
        case 1:
            printf("player red win\n");
            break;
        case 0:
            printf("draw\n");
            break;
        case -1:
            printf("player green win\n");
            break;
    }
    return 0;
}
