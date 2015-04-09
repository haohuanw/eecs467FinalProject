#ifndef _NAVIGATOR_HPP_
#define _NAVIGATOR_HPP_

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <functional>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <cassert>
#include <utility>
#include <cfloat>
#include "math/point.hpp"

struct vnode
{
    double x;
    double y;
    
    // array of indices of neighbors
    std::vector<int> neighbors;
};

struct line
{
    eecs467::Point<double> start;
    eecs467::Point<double> end;
    double m;
};

struct vnode_path
{
    double x;
    double y;
    int index;
    double cost;
    double est_cost;
    int came_from;
};

class vnode_comp
{
    public:
    bool operator()(vnode_path *a, vnode_path *b) const
    {
        return a->est_cost > b->est_cost;
    }
};

class Navigator
{
    private:
        std::vector<vnode> diagram;
        std::vector<line> diagram_lines;

        double dist(vnode_path *a, vnode_path *b);
        double heuristic(vnode_path *a, vnode_path *b);
        void remove(vnode_path *node, std::vector<vnode_path*>& open_set_vector);
        //void reconstructPath(vnode_path *end_node, std::vector<eecs467::Point<double>>& path);
        void reconstructPath(vnode_path *end_node, std::vector<eecs467::Point<double> >& path, std::vector<vnode_path>& nodes);
        void reverse(std::vector<eecs467::Point<double> >& path);
        std::pair<int, eecs467::Point<double>> findClosestLine(eecs467::Point<double> p);
        eecs467::Point<double> findClosestStart(eecs467::Point<double> p);
        std::pair<eecs467::Point<double>, eecs467::Point<double>> findClosestEnd(eecs467::Point<double> p);
        bool isSamePoint(eecs467::Point<double> a, eecs467::Point<double> b);

        bool vnodeIsEqual(vnode& a, eecs467::Point<double>& b);
        void printPath(std::vector<vnode_path>& nodes);
    public:
        Navigator(std::string filename);
        ~Navigator();
        std::vector<eecs467::Point<double>> pathPlan(eecs467::Point<double> start, eecs467::Point<double> end);
        void printDiagram();
        void printOpenSet(std::priority_queue<vnode_path*, std::vector<vnode_path*>, vnode_comp> open_set);
};

#endif
