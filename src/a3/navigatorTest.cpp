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
    //n.printDiagram();

    // RIGHT
    eecs467::Point<double> a = {-.6, -1.0};
    eecs467::Point<double> b = {-.4, 1.0};
    std::deque<eecs467::Point<double>> path = n.pathPlan(a, b);
    print(path);
    // (-0.5, -0.25) -> (-0.5, 1)

    // RIGHT
    a.x = -.5;
    a.y = -.25;
    path = n.pathPlan(a, b);
    print(path);
    // (-0.5, 1)

    // RIGHT
    b.x = -.5;
    b.y = -.125;
    path = n.pathPlan(a, b);
    print(path);
    // (-0.5, -0.125)
    
    // RIGHT
    b.x = .5;
    b.y = -.125;
    path = n.pathPlan(a, b);
    print(path);
    // (-0.5, -0.125) -> (0.375, -0.125) -> (0.5, -0.125)
    
    // RIGHT
    a.x = -0.7;
    a.y = 1.1;
    b.x = -0.6;
    b.y = -1.1;
    path = n.pathPlan(a, b);
    print(path);
    // (-0.75,0.25)->(-0.75,-0.125)->(-0.5,-0.125)->(0.375,-0.125)->(0.5,-0.125)->(0.5,-1.125)->(-0.5,-1.125)->(-0.5,-1.1)

    // RIGHT
    b.y = -0.125;
    path = n.pathPlan(a, b);
    print(path);
    // (-0.75,0.25)->(-0.75,-0.125)->(-0.6,-0.125)

    // RIGHT
    b.x = 0.75;
    path = n.pathPlan(a, b);
    print(path);
    // (-0.75,0.25)->(-0.75,-0.125)->(-0.5,-0.125)->(0.375,-0.125)->(0.5,-0.125)->(0.75,-0.125);

    // RIGHT
    a.x = 0.5;
    a.y = 0.25;
    b.x = -0.5;
    b.y = -0.25;
    path = n.pathPlan(a, b);
    print(path);
    // (0.5,-1.125)->(-0.5,-1.125)->(-0.5,-0.25)

    // RIGHT
    a.x = -0.5;
    a.y = 1.125;
    b.x = -0.75;
    b.y = 1.375;
    path = n.pathPlan(a, b);
    print(path);
    // (0.5,1.125)->(0.5,0.25)->(0.5,-1.125)->(-0.5,-1.125)->(-0.5,-0.25)->(-0.5,-0.125)->(0.375,-0.125)->(0.5,-0.125)->(0.75,-0.125)->(0.75,1.375)->(-0.75,1.375)

    // right
    a.x = 0.75;
    a.y = 0.125;
    b.x = -0.75;
    b.y = 0.125;
    path = n.pathPlan(a, b);
    print(path);
    // (0.5,0.125)->(-0.375,0.125)->(-0.5,0.125)->(-0.75,0.125)

    // RIGHT
    a.x = .75;
    a.y = -1.0;
    b.x = -.73;
    b.y = -1.0;
    path = n.pathPlan(a, b);
    print(path);
    // (0.75,-0.25)->(0.75,0.125)->(0.5,0.125)->(-0.375,0.125)->(-0.5,0.125)->(-0.75,0.125)0>(-0.75,-1)

    a.x = -0.6;
    a.y = 1.2;
    b.x = .5;
    b.y = 1.125;
    path = n.pathPlan(a, b);
    print(path);

    a.x = 0.0633;
    a.y = 1.282;
    b.x = -0.539;
    b.y = 0.6676;
    path = n.pathPlan(a, b);
    print(path);

    a.x = 0.46;
    a.y = -0.75;
    b.x = -0.5;
    b.y = -0.67;
    path = n.pathPlan(a, b);
    print(path);

    return 0;
}
