#include <unordered_map>
#include <queue>
#include <string>
#include <sstream>
#include <cmath>
#include <algorithm>
#include "mapping/occupancy_grid.hpp"
#include "mapping/occupancy_grid_utils.hpp"



using namespace std;

struct PriorityPoint
{
    int x;
    int y;
    int cost;
    int priority;
    PriorityPoint *parent;

    PriorityPoint(int _x=0, int _y=0, int _cost=0, int _priority=0) : x(_x), y(_y), cost(_cost), priority(_priority), parent(NULL) { }
    bool operator==(const PriorityPoint& b)
    {
        return (x == b.x && y == b.y && cost == b.cost && priority == b.priority && parent == b.parent);
    }
};

struct PriorityPointComparator
{
    bool operator()(PriorityPoint &a, PriorityPoint &b)
    {
        return a.priority > b.priority;
    }
};

class Path
{
    private:
        vector<PriorityPoint> open;
        unordered_map<string, PriorityPoint> closed;

        int startx;
        int starty;
        int endx;
        int endy;

        eecs467::OccupancyGrid *grid;

        bool failFlag;

        void checkPoint(PriorityPoint &_cur, int _x, int _y);
        bool valid(PriorityPoint &p) const;
        PriorityPoint openContains(PriorityPoint p);
        double movementCost(PriorityPoint &p, PriorityPoint &pp) const;
        double h(PriorityPoint &p) const;

    public:
        Path(eecs467::OccupancyGrid* _grid);
        void pathfind(int _startx, int _starty, int _endx, int _endy);
        void pathfind();
        void removeClosePoints(int _x, int _y);
        vector<double> x;
        vector<double> y;

        int size();
        double getCenterMetersX(int _x, double _scale);
        double getCenterMetersY(int _y, double _scale);
        double getNextCenterMetersX(double _scale);
        double getNextCenterMetersY(double _scale);
        int getCellPositionX(double _x, double _scale);
        int getCellPositionY(double _y, double _scale);
};
