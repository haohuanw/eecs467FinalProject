#ifndef GAMEBOARD_HPP
#define GAMEBOARD_HPP
#include <vector>
class gameboard{
    public:
        gameboard();
        void print_board();
        void update_board(int move,char pieces);
        void update_entire_board(std::vector <char> b);
        int  is_win(char pieces);//1 pieces win, 0 draw, -1 pieces lose
        bool is_valid(char piece1,char piece2);
        bool is_finished();
        std::vector<int> get_board(char pieces);
        bool operator==(const gameboard& rhs);
        std::vector<char> board;
    private:
        char print_grid(int i);
};

#endif
