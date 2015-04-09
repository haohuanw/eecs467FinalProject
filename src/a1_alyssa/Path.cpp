#include "Path.h"
#include <iostream>
#include <cassert>

using namespace std;

Path::Path(eecs467::OccupancyGrid *_grid) : grid(_grid)
{
}

void Path::pathfind(int _startx, int _starty, int _endx, int _endy)
{
    open.clear();
    closed.clear();
    failFlag = false;
    x.clear();
    y.clear();
    PriorityPoint p(_startx, _starty, 0, -1);
    open.push_back(p);
    startx = _startx;
    starty = _starty;
    endx = _endx;
    endy = _endy;
    pathfind();
}

void Path::pathfind()
{
	while(!open.empty())
	{
		PriorityPoint cur = open.back();
        open.pop_back();
		string s;
		stringstream ss;
		ss << cur.x << ',' << cur.y;
		s = ss.str();
		closed[s] = cur;

		if(cur.x == endx && cur.y == endy)
		{
            std::cout << "begin while cur.parent != NULL loop" << std::endl;
			while(cur.parent != NULL)
			{
				x.push_back(cur.x);
				y.push_back(cur.y);
                assert(&cur != cur.parent);
                cur = *cur.parent;
			}
            std::cout << "while cur.parent != NULL loop" << std::endl;
			return;
		}
		else
		{
			checkPoint(cur, cur.x-1, cur.y-1);
			checkPoint(cur, cur.x-1, cur.y);
			checkPoint(cur, cur.x-1, cur.y+1);
			checkPoint(cur, cur.x, cur.y-1);
			checkPoint(cur, cur.x, cur.y+1);
			checkPoint(cur, cur.x+1, cur.y-1);
			checkPoint(cur, cur.x+1, cur.y);
			checkPoint(cur, cur.x+1, cur.y+1);
		}
	}
	failFlag = true;
}

double Path::getCenterMetersX(int _x, double _scale)
{
    return (_x + 0.5) * _scale;
}
double Path::getCenterMetersY(int _y, double _scale)
{
    return (_y * 0.5) * _scale;
}
double Path::getNextCenterMetersX(double _scale)
{
    if(size() <= 0) return -1;
    return getCenterMetersX(x.back(), _scale);
}
double Path::getNextCenterMetersY(double _scale)
{
    if(size() <= 0) return -1;
    return getCenterMetersY(y.back(), _scale);
}
int Path::getCellPositionX(double _x, double _scale)
{
    return (int)(_x / _scale);
}
int Path::getCellPositionY(double _y, double _scale)
{
    return (int)(_y / _scale);
}

int Path::size()
{
    return x.size();
}

void Path::removeClosePoints(int _x, int _y)
{
    if(size() <= 0)
        return;

    if(_x == x.back() && _y == y.back())
    {
        x.pop_back();
        y.pop_back();
    }
}

void Path::checkPoint(PriorityPoint &_cur, int _x, int _y)
{
    if(grid->logOdds(_x, _y) > 0 || !grid->isCellInGrid(_x, _y))
        return;

	PriorityPoint *neighbor = new PriorityPoint(_x, _y);
	neighbor->cost = _cur.cost + movementCost(_cur, *neighbor);
	neighbor->priority = -neighbor->cost - h(*neighbor);
	
	PriorityPoint op = openContains(*neighbor);
	if(valid(op) && neighbor->cost < op.cost)
	{
        vector<PriorityPoint>::iterator it = std::find(open.begin(), open.end(), op);
		open.erase(it);
		return;
	}

	string s;
	stringstream ss;
	ss << _x << ',' << _y;
	s = ss.str();
	bool c_exists = closed.count(s) > 0;
	if(c_exists)
	{
		PriorityPoint cp = closed[s];
		if(neighbor->cost < cp.cost)
		{
			closed.erase(s);
			return;
		}
	}

	if(!valid(op) && !c_exists)
	{
		neighbor->parent = &_cur;
		assert(neighbor!=&_cur);
        open.push_back(*neighbor);
        PriorityPointComparator comp;
        std::make_heap(open.begin(), open.end(), comp);
	}
}

bool Path::valid(PriorityPoint &p) const
{
	return p.x >= 0 && p.y >= 0;
}

PriorityPoint Path::openContains(PriorityPoint p)
{
    std::cout << "begin openContains" << std::endl;
	for(uint i = 0; i < open.size(); i++)
	{
		if(open[i].x == p.x && open[i].y == p.y)
			return open[i];
	}
    std::cout << "end openContains" << std::endl;

	PriorityPoint pp(-1, -1);
	return pp;
}

double Path::movementCost(PriorityPoint &p, PriorityPoint &pp) const 
{
	return abs(p.x - pp.x) + abs(p.y - pp.y);
}

double Path::h(PriorityPoint &p) const
{
	return abs(p.x - endx) + abs(p.y - endy);
}
