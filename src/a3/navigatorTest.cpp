#include "Navigator.hpp"
#include <iostream>

using namespace std;

void print(std::vector<point>& path)
{
    for(uint i = 0; i < path.size(); i++)
    {
        std::cout << "(" << path[i].x << ", " << path[i].y << ") ->";
    }
    std::cout << std::endl;
}

int main()
{
    Navigator n("../ground_truth/vmap.txt");
    //n.printDiagram();

    point a = {-0.75, 1.375};
    point b = {-0.75, 0.25};
    std::vector<point> path = n.pathPlan(a, b);
    print(path);

    b.y = -0.125;
    std::vector<point> path2 = n.pathPlan(a, b);
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
    print(path2);

    return 0;
}
