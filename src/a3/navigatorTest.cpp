#include "Navigator.hpp"
#include <iostream>

using namespace std;

void print(std::deque<eecs467::Point<double>>& path)
{
    for(uint i = 0; i < path.size(); i++)
    {
        std::cout << "(" << path[i].x << ", " << path[i].y << ") ->";
    }
    std::cout << std::endl;
}

/*void printPoint(std::pair<int, eecs467::Point<double>>& p)
{
    std::cout << "(" << p.first << ", " << p.second.x << ", " << p.second.y << ")\n";
}*/

int main()
{
    Navigator n("../ground_truth/vmap.txt");
    n.printDiagram();

    eecs467::Point<double> a = {-.41945, -.817545};
    eecs467::Point<double> b = {-.5, 0.886755};
    std::deque<eecs467::Point<double>> path = n.pathPlan(a, b);
    print(path);

    /*b.y = 0.25;
    path = n.pathPlan(a, b);
    print(path);

    a.x = -0.7;
    a.y = 1.1;
    b.x = -0.6;
    b.y = -1.1;
    path = n.pathPlan(a, b);
    print(path);*/

    /*b.y = -0.125;
    std::vector<eecs467::Point<double>> path2 = n.pathPlan(a, b);
    print(path2);

    b.x = 0.75;
    path2 = n.pathPlan(a, b);
    print(path2);

    a.x = 0.5;
    a.y = 0.25;
    b.x = -0.5;
    b.y = -0.25;
    path2 = n.pathPlan(a, b);
    print(path2);

    a.x = -0.5;
    a.y = 1.125;
    b.x = -0.75;
    b.y = 1.375;
    path2 = n.pathPlan(a, b);
    print(path2);

    a.x = 0.75;
    a.y = 0.125;
    b.x = -0.75;
    b.y = 0.125;
    path2 = n.pathPlan(a, b);
    print(path2);*/

    /*a.x = 0;
    a.y = 1.35;
    std::pair<int, eecs467::Point<double>> retval;
    retval = n.findClosestLine(a);
    printPoint(retval);

    a.x = 0;
    a.y = 0.1;
    retval = n.findClosestLine(a);
    printPoint(retval);

    a.x = -0.5;
    a.y = -0.12;
    retval = n.findClosestLine(a);
    printPoint(retval);

    a.x = -0.5625;
    a.y = -0.125 - 0.0626;
    retval = n.findClosestLine(a);
    printPoint(retval);*/

    return 0;
}
