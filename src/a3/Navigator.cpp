#include "Navigator.hpp"

Navigator::Navigator(std::string filename)
{
    std::ifstream input(filename.c_str());
    if(!input.is_open())
    {
        std::cout << "error opening file" << std::endl;
        exit(1);
    }
    int numNodes;
    input >> numNodes;
    diagram.resize(numNodes);
    for(int i = 0; i < numNodes; i++)
    {
        double x, y;
        input >> x >> y;
        diagram[i].x = x;
        diagram[i].y = y;
    }

    int index1, index2;
    while(input >> index1 >> index2)
    {
        diagram[index1].neighbors.push_back(index2);
    }
}

Navigator::~Navigator()
{
}

std::vector<point> Navigator::pathPlan(point start, point end)
{
    vnode_path *start_node, *end_node;
    // TODO: implement find_closest_end
    //vnode closest_end = find_closest_end(end);
    point closest_end = end;
    std::vector<vnode_path> nodes;
    nodes.resize(diagram.size());
    for(uint i = 0; i < diagram.size(); i++)
    {
        vnode_path v = {diagram[i].x, diagram[i].y, i, HUGE_VAL, HUGE_VAL, -1};
        nodes[i] = v;
        if(vnodeIsEqual(diagram[i], start)) { start_node = &nodes[i]; }
        if(vnodeIsEqual(diagram[i], closest_end)) { end_node = &nodes[i]; }
    }
    
    std::unordered_set<vnode_path*> closed_set;
    std::priority_queue<vnode_path*, std::vector<vnode_path*>, std::greater<vnode_path*>> open_set;
    std::vector<vnode_path*> open_set_vector;
    
    start_node->cost = 0;
    start_node->est_cost = dist(start_node, end_node);
    open_set.push(start_node);
    open_set_vector.push_back(start_node);

    while(!open_set.empty())
    {
        vnode_path *current = open_set.top();
        if(current == end_node)
        {
            std::vector<point> path;
            reconstructPath(current, path, nodes);
            return path;
        }
        open_set.pop();
        remove(current, open_set_vector); // remove current from open_set_vector
        closed_set.insert(current);
        for(uint i = 0; i < diagram[current->index].neighbors.size(); i++)
        {
            vnode_path *neighbor = &nodes[diagram[current->index].neighbors[i]];
            if(closed_set.find(neighbor) != closed_set.end())
            {
                continue;
            }
            double tentative_cost = current->cost + dist(current, neighbor);
            if(std::find(open_set_vector.begin(), open_set_vector.end(), neighbor) == open_set_vector.end()
               || tentative_cost < neighbor->cost)
            {
                neighbor->came_from = current->index;
                neighbor->cost = tentative_cost;
                neighbor->est_cost = neighbor->cost + heuristic(neighbor, end_node);
                if(std::find(open_set_vector.begin(), open_set_vector.end(), neighbor) == open_set_vector.end())
                {
                    open_set.push(neighbor);
                    open_set_vector.push_back(neighbor);
                } // if
            } // if
        } // for
    } // while
    std::vector<point> retval;
    return retval;
}

double Navigator::dist(vnode_path *a, vnode_path *b)
{
    return sqrt((b->x - a->x)*(b->x - a->x) + (b->y - a->y)*(b->y - a->y));
}

double Navigator::heuristic(vnode_path *a, vnode_path *b)
{
    return dist(a, b);
}

/* removes node from open_set_vector */
void Navigator::remove(vnode_path *node, std::vector<vnode_path*>& open_set_vector)
{
    std::vector<vnode_path*>::iterator it = std::find(open_set_vector.begin(), open_set_vector.end(), node);
    *it = open_set_vector[open_set_vector.size() - 1];
    open_set_vector.pop_back();
}

//void Navigator::reconstructPath(vnode_path *end_node, point end, std::vector<point>& path)
void Navigator::reconstructPath(vnode_path *end_node, std::vector<point>& path, std::vector<vnode_path>& nodes)
{
    vnode_path *current = end_node;
    std::cout << "end node: " << end_node->x << ", " << end_node->y << std::endl;
    point next_coord = {end_node->x, end_node->y};
    printPath(nodes);
    while(current->came_from != -1)
    {
        next_coord.x = current->x;
        next_coord.y = current->y;
        path.push_back(next_coord);
        current = &nodes[current->came_from];
    }

    // NOTE: if start point is implemented as the closest point BEHIND where you are now,
    //       take these three lines out!!
    next_coord.x = current->x;
    next_coord.y = current->y;
    path.push_back(next_coord);
    reverse(path);
}

void Navigator::printPath(std::vector<vnode_path>& nodes)
{
    for(uint i = 0; i < nodes.size(); i++)
    {
        std::cout << "nodes[" << i << "]: {" << nodes[i].x << ", " << nodes[i].y << ", "
                  << nodes[i].index << ", " << nodes[i].cost << ", " << nodes[i].est_cost
                  << ", " << nodes[i].came_from << "}\n";
    }
}

void Navigator::reverse(std::vector<point>& path)
{
    for(uint i = 0; i < path.size()/2; i++)
    {
        std::swap(path[i], path[path.size() - 1 - i]);
    }
}

bool Navigator::vnodeIsEqual(vnode& a, point& b)
{
    return a.x == b.x && a.y == b.y;
}

void Navigator::printDiagram()
{
    std::cout << "Nodes: " << std::endl;
    for(uint i = 0; i < diagram.size(); i++)
    {
        for(uint j = 0; j < diagram[i].neighbors.size(); j++)
        {
            std::cout << "(" << diagram[i].x << ", " << diagram[i].y << ") -> ("
                      << diagram[diagram[i].neighbors[j]].x << ", "
                      << diagram[diagram[i].neighbors[j]].y << ")" << std::endl;
        }
    }
}
