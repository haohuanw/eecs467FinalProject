#include "OccupancyGridMapper.hpp"
#include <iostream>
#include <cassert>

OccupancyGridMapper::OccupancyGridMapper(lcm::LCM *lcm_t) :
	bfs_(10, 10, 0.05),
	occupancy_grid_(10, 10, 0.05),
	occupancy_grid_expanded_(10, 10, 0.05),
    lcm(lcm_t),
    count(0)
{
    pthread_mutex_init(&mapper_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);
    //end_points_.reserve(300);
}

/*OccupancyGridMapper::OccupancyGridMapper(int height, int width, double cellSize) :
    occupancy_grid_(width, height, cellSize)
{
    pthread_mutex_init(&mapper_mutex_, NULL);
    pthread_cond_init(&cv_, NULL);
}*/

OccupancyGridMapper::~OccupancyGridMapper()
{
}

void OccupancyGridMapper::setLogOddsMapper(int x, int y, double logOdds)
{
    occupancy_grid_(x, y) = logOdds;
}

void OccupancyGridMapper::clearBFS()
{
    for(uint y = 0; y < bfs_.heightInCells(); y++)
    {
        for(uint x = 0; x < bfs_.widthInCells(); x++)
        {
            bfs_(x, y) = 0;
        }
    }
}

void OccupancyGridMapper::resetBFS()
{
    for(uint y = 0; y < bfs_.heightInCells(); y++)
    {
        for(uint x = 0; x < bfs_.widthInCells(); x++)
        {
            auto v = occupancy_grid_expanded_(x, y);
            bfs_(x, y) = 0;
            if(v > 0)
            {
                setOccupied(x, y);
            }
            else if (v < 0)
            {
                setFree(x, y);
            }
            else
            {
                setUnknown(x, y);
            }
        }
    }
}

std::vector<int> OccupancyGridMapper::search(int x, int y)
{
    std::queue<int> q;
    q.push(toIndex(x, y));
    //int count = 0;
    while(!q.empty())
    {
        auto top =q.front();
        int topx = toX(top);
        int topy = toY(top);
        q.pop();
        std::queue<int> neighbors;
        if(!getVisited(topx-1, topy))
        {
            neighbors.push(toIndex(topx-1, topy));
            setParentRight(topx-1, topy);
        }
        if(!getVisited(topx-1, topy+1))
        {
            neighbors.push(toIndex(topx-1, topy+1));
            setParentUp(topx-1, topy+1);
            setParentRight(topx-1, topy+1);
        }
        if(!getVisited(topx-1, topy-1))
        {
            neighbors.push(toIndex(topx-1, topy-1));
            setParentDown(topx-1, topy-1);
            setParentRight(topx-1, topy-1);
        }
        if(!getVisited(topx, topy+1))
        {
            neighbors.push(toIndex(topx, topy+1));
            setParentUp(topx, topy+1);
        }
        if(!getVisited(topx, topy-1))
        {
            neighbors.push(toIndex(topx, topy-1));
            setParentDown(topx, topy-1);
        }
        if(!getVisited(topx+1, topy))
        {
            neighbors.push(toIndex(topx+1, topy));
            setParentLeft(topx+1, topy);
        }
        if(!getVisited(topx+1, topy+1))
        {
            neighbors.push(toIndex(topx+1, topy+1));
            setParentLeft(topx+1, topy+1);
            setParentUp(topx+1, topy+1);
        }
        if(!getVisited(topx+1, topy-1))
        {
            neighbors.push(toIndex(topx+1, topy-1));
            setParentLeft(topx+1, topy-1);
            setParentDown(topx+1, topy-1);
        }
        while(!neighbors.empty())
        {
            int next = neighbors.front();
            neighbors.pop();
            int nextx = toX(next);
            int nexty = toY(next);
            setVisited(nextx, nexty);
            if(!getOccupied(nextx, nexty))
            {
                if(getUnknown(nextx, nexty))
                {
                    return backtrace(nextx, nexty, x, y);
                }
                q.push(next);
            }
        }
        /*if(count == 0)
        {
            maebot_occupancy_grid_t data = bfs_.toLCM();
            lcm->publish("OCCUPANCY_GRID_GUI", &data);
        }
        count = (count + 1) % 20;
        std::cout << "inner while" << std::endl;*/
    }
    std::vector<int> retval;
    return retval;
}

int OccupancyGridMapper::toIndex(int x, int y){
    return y*bfs_.widthInCells() + x;
}

int OccupancyGridMapper::toX(int index){
    return index % bfs_.widthInCells();
}

int OccupancyGridMapper::toY(int index){
    return index / bfs_.widthInCells();
}

void OccupancyGridMapper::toXYinMeters(int index, double &xm, double &ym)
{
    eecs467::Point<double> pt(index % bfs_.widthInCells(), index / bfs_.widthInCells());
    pt = eecs467::grid_position_to_global_position(pt, occupancy_grid_);
    xm = pt.x;
    ym = pt.y;
}

std::vector<int> OccupancyGridMapper::backtrace(int endx, int endy, int startx, int starty)
{ 
    std::vector<int> r;
    int count = 0;
    while(1)
    {
        assert(r.size() <= 40000);
        if(count%4)
        {
            r.push_back(toIndex(endx, endy));
        }
        int newx = (int)getParentRight(endx, endy) - (int)getParentLeft(endx, endy) + endx;
        int newy = (int)getParentDown(endx, endy) - (int)getParentUp(endx, endy) + endy;

        if(newx == startx && newy == starty) { break; }
        endx = newx;
        endy = newy;
        count++;
    }
    for(uint i = 0; i < r.size(); i++)
    {
        //eecs467::Point<int> c(toX(r[i]), toY(r[i]));
        //auto cellPoint = global_position_to_grid_cell(c, occupancy_grid_);
        maebot_pose_t pose;
        //pose.x = cellPoint.x;
        //pose.y = cellPoint.y;
        pose.x = (toX(r[i])+0.5)*occupancy_grid_.metersPerCell() - occupancy_grid_.heightInMeters()/2;
        pose.y = (toY(r[i])+0.5)*occupancy_grid_.metersPerCell() - occupancy_grid_.widthInMeters()/2;
        lcm->publish("MAEBOT_PATH_GUI", &pose);
        //std::cout << "publish path" << std::endl;
    }
    return r;
}

bool OccupancyGridMapper::getUnknown(int _x, int _y)
{
    return isolateBit(bfs_(_x, _y), 5);
}

bool OccupancyGridMapper::getOccupied(int _x, int _y)
{
    return isolateBit(bfs_(_x, _y), 6);// || isolateBit(bfs(_x, _y), 7) if want unknown to count as occupied
}

bool OccupancyGridMapper::getVisited(int _x, int _y)
{
    return isolateBit(bfs_(_x, _y), 7);
}

void OccupancyGridMapper::setParentRight(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 1;
}
bool OccupancyGridMapper::getParentRight(int x, int y)
{
    return bfs_(x, y) & 1;
}

void OccupancyGridMapper::setParentUp(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 2;
}
bool OccupancyGridMapper::getParentUp(int x, int y)
{
    return isolateBit((bfs_(x, y) & 2), 1);
}

void OccupancyGridMapper::setParentLeft(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 4;
}
bool OccupancyGridMapper::getParentLeft(int x, int y)
{
    return isolateBit((bfs_(x,y) & 4), 2);
}

void OccupancyGridMapper::setParentDown(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 8;
}
bool OccupancyGridMapper::getParentDown(int x, int y)
{
    return isolateBit((bfs_(x, y) & 8), 3);
}

void OccupancyGridMapper::setFree(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 16;
}

void OccupancyGridMapper::setUnknown(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 32;
}

void OccupancyGridMapper::setOccupied(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 64;
}

void OccupancyGridMapper::setVisited(int x, int y)
{
    bfs_(x, y) = bfs_(x, y) | 128;
}

bool OccupancyGridMapper::isolateBit(int v, int index)
{
    return (v >> index) & 1;
}

void OccupancyGridMapper::expandOccupancyGrid()
{
    int w = occupancy_grid_.widthInCells();
    int h = occupancy_grid_.heightInCells();
    std::unordered_set<int> s;
    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            auto v = occupancy_grid_(x, y);
            occupancy_grid_expanded_(x, y) = v;
            if(v > 0)
            {
                for(int y0 = y - EXPAND_THICKNESS; y0 <= y+EXPAND_THICKNESS; y0++)
                {
                    for(int x0 = x - EXPAND_THICKNESS; x0 <= x + EXPAND_THICKNESS; x0++)
                    {
                        s.insert(y0*w + x0);
                    }
                }
            }
        }
    }

    for(const int &i: s)
    {
        occupancy_grid_expanded_.setLogOdds(i%w, i/w, 127);
    }
}

LaserScan OccupancyGridMapper::calculateLaserOrigins()
{
    maebot_pose_t pose = poses_.front();
    poses_.pop();
    maebot_laser_scan_t laser_scan = laser_scans_.front();
    laser_scans_.pop();

    if(poses_.empty())
    {
        approx_laser_.addPose(&pose);
        LaserScanRange lsr(true, pose, pose, laser_scan);

        LaserScan ls = moving_laser_.findOrigin(lsr);

        return ls;
    }
    else
    {
        assert(laser_scan.utime == pose.utime);

        if(!approx_laser_.containsPoses())
        {
            approx_laser_.addPose(&pose);
            LaserScan ls;
            ls.valid = false;
            return ls;
        }

        approx_laser_.addPose(&pose);
        LaserScanRange lsr = approx_laser_.findPts(&laser_scan);
        if(lsr.valid == false)
        {
            LaserScan ls;
            ls.valid = false;
            return ls;
        }

        LaserScan ls = moving_laser_.findOrigin(lsr);

        return ls;
    }
}

std::vector<int> OccupancyGridMapper::updateGrid(LaserScan scan)
{
    for(unsigned int i = 0; i < scan.origins.size(); ++i)
    {
        double a = scan.origins[i].theta - scan.scan.thetas[i];
        double d = scan.scan.ranges[i];
        double e_x = scan.origins[i].x + d * cos(a);
        double e_y = scan.origins[i].y + d * sin(a);
        //maebot_pose_t ep;
        //ep.x = e_x;
        //ep.y = e_y;
        //ep.utime = scan.scan.utime;
        //end_points_[i] = ep;
        drawLineMeters(scan.origins[i].x, scan.origins[i].y, e_x, e_y, -2, 1);
    }
    expandOccupancyGrid();
    eecs467::Point<double> c(scan.end_pose.x, scan.end_pose.y);
    eecs467::Point<int> cellPoint = global_position_to_grid_cell(c, occupancy_grid_);
    
    if(count == 0)
    {
        resetBFS();
        bfs_result_ = search(cellPoint.x, cellPoint.y);
    }
    count = (count+1)%50;
    return bfs_result_;
}

/*
	s_x, s_y :	start position of the line (meters)
	e_x, e_y : end position of the line (meters)
	inc		 : amount to travel along the line per loop (meters)
	value 	 : value to set for all cells along the line
*/
void OccupancyGridMapper::drawLineMeters(double s_x, double s_y, double e_x, double e_y, double inc, eecs467::CellOdds value, eecs467::CellOdds value_end)
{
	double a = atan2(e_y - s_y, e_x - s_x);
    double dist = sqrt((e_x - s_x)*(e_x - s_x)+(e_y - s_y)*(e_y - s_y));
    if(dist == 0)
    {
        return;
    }
	int n = ceil(dist / inc);
    inc = dist/n;
	double i_x = cos(a) * inc,
		   i_y = sin(a) * inc,
		   p_x = s_x,
		   p_y = s_y;

    eecs467::Point<double> last_cell(e_x, e_y);
    last_cell = eecs467::global_position_to_grid_cell(last_cell, occupancy_grid_);
		   
	for(int i = 0; i <= n; i++)
	{
        int v;
        eecs467::Point<double> p(p_x, p_y);
		auto p_origin = eecs467::global_position_to_grid_cell(p, occupancy_grid_);
        if(p_origin == last_cell)
        {
            v = (int)occupancy_grid_.logOdds(p_origin.x, p_origin.y) + (int)value_end;
        }
        else
        {
            v = (int)occupancy_grid_.logOdds(p_origin.x, p_origin.y) + (int)value;
        }
        v = v > 127 ? 127 : (v < -127 ? -127 : v);
        occupancy_grid_.setLogOdds(p_origin.x, p_origin.y, v);
			
		p_x += i_x;
		p_y += i_y;
	}
}
//Calls with a default increment of 1 cell
void OccupancyGridMapper::drawLineMeters(double s_x, double s_y, double e_x, double e_y, eecs467::CellOdds value, eecs467::CellOdds value_end)
{
	drawLineMeters(s_x, s_y, e_x, e_y, occupancy_grid_.metersPerCell(), value, value_end);
}

void OccupancyGridMapper::publishOccupancyGrid(maebot_pose_t pose)
{
    //maebot_occupancy_grid_t data = occupancy_grid_expanded_.toLCM();
    maebot_occupancy_grid_t data = occupancy_grid_.toLCM();
    //maebot_occupancy_grid_t data = bfs_.toLCM();
    if(!data.utime)data.utime=0;

    lcm->publish("OCCUPANCY_GRID_GUI", &data);
    /*int64_t time = end_points_[0].utime;
    int i = 0;
    while(end_points_[i].utime == time)
    {
        lcm->publish("MAEBOT_LASER_ENDPOINTS", &end_points_[i]);
        i++;
    }*/
    lcm->publish("MAEBOT_POSE_BEST", &pose);
    //std::cout << "sent grid\n";
}

ApproxLaser OccupancyGridMapper::getApproxLaser()
{
    return approx_laser_;
}

MovingLaser OccupancyGridMapper::getMovingLaser() {
    return moving_laser_;
}

eecs467::OccupancyGrid& OccupancyGridMapper::getOccupancyGrid() {
    return occupancy_grid_;
}

bool OccupancyGridMapper::laserScansEmpty()
{
    return laser_scans_.empty();
}

bool OccupancyGridMapper::posesEmpty()
{
    return poses_.empty();
}

void OccupancyGridMapper::lockMapperMutex()
{
    pthread_mutex_lock(&mapper_mutex_);
}

void OccupancyGridMapper::unlockMapperMutex()
{
    pthread_mutex_unlock(&mapper_mutex_);
}

void OccupancyGridMapper::wait()
{
    pthread_cond_wait(&cv_, &mapper_mutex_);
}

void OccupancyGridMapper::signal()
{
    pthread_cond_signal(&cv_);
}

void OccupancyGridMapper::addLaserScan(maebot_laser_scan_t input_scan)
{
    laser_scans_.push(input_scan);
}

void OccupancyGridMapper::addPose(maebot_pose_t input_pose)
{
    poses_.push(input_pose);
}

double OccupancyGridMapper::metersPerCellMapper()
{
    return occupancy_grid_.metersPerCell();
}
