#ifndef ARM_AI_HPP
#define ARM_AI_HPP
#include <vector>

class arm_ai{
    public:
	arm_ai(){}
        arm_ai(char p);    
        int calc_move(std::vector<int> board);//didn't modify board
        int is_win(const std::vector<int> board);
        char get_player();
    private:
        int min_max(std::vector<int> &board,int player);//didn't modify board
        char player;// 1 or -1
};

#endif
