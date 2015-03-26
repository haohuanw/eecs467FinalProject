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

struct vnode
{
    double x;
    double y;
    
    // array of indices of neighbors
    std::vector<int> neighbors;
};

struct point
{
    double x;
    double y;
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

class Navigator
{
    private:
        std::vector<vnode> diagram;

        double dist(vnode_path *a, vnode_path *b);
        double heuristic(vnode_path *a, vnode_path *b);
        void remove(vnode_path *node, std::vector<vnode_path*>& open_set_vector);
        //void reconstructPath(vnode_path *end_node, std::vector<point>& path);
        void reconstructPath(vnode_path *end_node, std::vector<point>& path, std::vector<vnode_path>& nodes);
        void reverse(std::vector<point>& path);

        bool vnodeIsEqual(vnode& a, point& b);
        void printPath(std::vector<vnode_path>& nodes);
    public:
        Navigator(std::string filename);
        ~Navigator();
        std::vector<point> pathPlan(point start, point end);
        void printDiagram();
};

#endif
