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
        line l = {{diagram[index1].x, diagram[index1].y}, {diagram[index2].x, diagram[index2].y}, 0};
        l.m = ((l.start.x == l.end.x) ? HUGE_VAL : 0);
        diagram_lines.push_back(l);
    }
}

Navigator::~Navigator()
{
}

std::deque<eecs467::Point<double>> Navigator::pathPlan(eecs467::Point<double> real_start, eecs467::Point<double> end)
{
    vnode_path *start_node, *end_node;
    std::pair<eecs467::Point<double>, eecs467::Point<double>> end_pair = findClosestEnd(end);
    eecs467::Point<double> start = findClosestStart(real_start);
    std::vector<vnode_path> nodes;
    nodes.resize(diagram.size());
    for(uint i = 0; i < diagram.size(); i++)
    {
        vnode_path v = {diagram[i].x, diagram[i].y, i, HUGE_VAL, HUGE_VAL, -1};
        nodes[i] = v;
        if(vnodeIsEqual(diagram[i], start)) { start_node = &nodes[i]; }
        if(vnodeIsEqual(diagram[i], end_pair.first)) { end_node = &nodes[i]; }
    }
    
    std::unordered_set<vnode_path*> closed_set;
    std::priority_queue<vnode_path*, std::vector<vnode_path*>, vnode_comp> open_set;
    std::vector<vnode_path*> open_set_vector;
    
    start_node->cost = 0;
    start_node->est_cost = dist(start_node, end_node);
    open_set.push(start_node);
    open_set_vector.push_back(start_node);

    while(!open_set.empty())
    {
        printOpenSet(open_set);
        vnode_path *current = open_set.top();
        if(current == end_node)
        {
            std::deque<eecs467::Point<double>> path;
            reconstructPath(current, path, nodes);
            path.push_back(end_pair.second);
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
    std::deque<eecs467::Point<double>> retval;
    return retval;
}

eecs467::Point<double> Navigator::findClosestStart(eecs467::Point<double> p)
{
    std::pair<int, eecs467::Point<double>> retval = findClosestLine(p);
    if(isSamePoint(diagram_lines[retval.first].start, p)) return p;
    return diagram_lines[retval.first].end;
}

// retval: closest node, actual endpoint
std::pair<eecs467::Point<double>, eecs467::Point<double>> Navigator::findClosestEnd(eecs467::Point<double> p)
{
    std::pair<int, eecs467::Point<double>> retval = findClosestLine(p);
    if(isSamePoint(diagram_lines[retval.first].end, p)) return std::make_pair(p, p);
    return std::make_pair(diagram_lines[retval.first].start, retval.second);
}

bool Navigator::isSamePoint(eecs467::Point<double> a, eecs467::Point<double> b)
{
    return a.x == b.x && a.y == b.y;
}

std::pair<int,eecs467::Point<double>> Navigator::findClosestLine(eecs467::Point<double> p)
{
    // find distances from point to lines
    std::vector<double> distances(diagram_lines.size(), -1);
    for(uint i = 0; i < diagram_lines.size(); i++)
    {
        if(diagram_lines[i].m == 0) // horizontal line
        {
            // if p.x is between start and end points of diagram_lines[i]
            if((p.x >= diagram_lines[i].start.x && p.x <= diagram_lines[i].end.x) ||
               (p.x <= diagram_lines[i].start.x && p.x >= diagram_lines[i].end.x))
            {
                eecs467::Point<double> intersect = {p.x, diagram_lines[i].start.y};
                distances[i] = fabs(intersect.y - p.y);
            }
        }
        else // vertical line
        {
            if((p.y >= diagram_lines[i].start.y && p.y <= diagram_lines[i].end.y) ||
               (p.y <= diagram_lines[i].start.y && p.y >= diagram_lines[i].end.y))
            {
                eecs467::Point<double> intersect = {diagram_lines[i].start.x, p.y};
                distances[i] = fabs(intersect.x - p.x);
            }
        }
    }

    // find smallest distance
    double dist = DBL_MAX;
    int index = -1;
    for(uint i = 0; i < distances.size(); i++)
    {
        if(distances[i] < dist && distances[i] != -1)
        {
            dist = distances[i];
            index = i;
        }
    }
    assert(index != -1);
    eecs467::Point<double> intersect;
    if(diagram_lines[index].m == 0)
    {
        intersect.x = p.x;
        intersect.y = diagram_lines[index].start.y;
    }
    else
    {
        intersect.x = diagram_lines[index].start.x;
        intersect.y = p.y;
    }
    std::pair<int, eecs467::Point<double>> retval(index, intersect);
    //std::cout << "distance: " << dist << std::endl;
    //std::cout << "index:    " << index << std::endl;
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
void Navigator::reconstructPath(vnode_path *end_node, std::deque<eecs467::Point<double>>& path, std::vector<vnode_path>& nodes)
{
    vnode_path *current = end_node;
    //std::cout << "end node: " << end_node->x << ", " << end_node->y << std::endl;
    eecs467::Point<double> next_coord = {end_node->x, end_node->y};
    //printPath(nodes);
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

void Navigator::reverse(std::vector<eecs467::Point<double>>& path)
{
    for(uint i = 0; i < path.size()/2; i++)
    {
        std::swap(path[i], path[path.size() - 1 - i]);
    }
}

bool Navigator::vnodeIsEqual(vnode& a, eecs467::Point<double>& b)
{
    return a.x == b.x && a.y == b.y;
}

void Navigator::printOpenSet(std::priority_queue<vnode_path*, std::vector<vnode_path*>, vnode_comp> open_set)
{
    std::cout << "CURRENT OPEN SET: " << std::endl;
    while(!open_set.empty())
    {
        vnode_path *next = open_set.top();
        open_set.pop();
        std::cout << "{x: " << next->x << ", y: " << next->y << ", index: " << next->index << ", cost: " << next->cost << ", est_cost: " << next->est_cost << "}\n";
    }
    std::cout << std::endl << "-----------------------------------------------------------------------------------------" << std::endl << std::endl;
}

void Navigator::printDiagram()
{
    // print nodes
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

    // print lines
    std::cout << "\nLines: \n";
    for(uint i = 0; i < diagram_lines.size(); i++)
    {
        std::cout << "{(" << diagram_lines[i].start.x << ", " << diagram_lines[i].start.y
                  << "), (" << diagram_lines[i].end.x << ", " << diagram_lines[i].end.y
                  << "), " << diagram_lines[i].m << "}\n";
    }
}
