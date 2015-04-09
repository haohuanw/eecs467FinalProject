#include "ApproxLaser.hpp"
#include <cassert>
#include <iostream>

LaserScanRange ApproxLaser::findPts(const maebot_laser_scan_t *scan)
{
    if(poses_.empty()){
        std::cout << "waiting for initial pose" << std::endl;
        exit(1); // if we called this without adding a pose first, we dun fucked up
    }

    // scrap the first lidar scan because we don't have two poses to interpolate from
    maebot_pose_t start, end;
    start.x = 0;
    start.y = 0;
    start.theta = 0;
    start.utime = 0;
    end = start;
    LaserScanRange retval = {false, start, end, *scan};
    if(poses_.size() != 1){
        bool found = false;
        std::deque<maebot_pose_t>::iterator iter;
        int i = 0;
        //std::cout << "scan time: " << scan->utime << std::endl;
        for(iter = poses_.begin(); iter != poses_.end(); ++iter)
        {
            //std::cout << "poses_[" << i << "]: " << iter->utime << std::endl;
            // the end time (scan->utime) of the lidar scan should be equal to the
            // utime of the pose
            if(iter->utime == scan->utime)
            {
                end = *iter;
                found = true;
                break;
            }
            i++;
        }
        // assert that range of poses is correct
        if(!found)
        {
            return retval;
        }
        assert((iter-1) >= poses_.begin());

        start = *(iter-1);
        retval.valid = true;
        retval.start_pose = start;
        retval.end_pose = end;

        //std::cout << "found start time: " << retval.start_pose.utime << std::endl;
        //std::cout << "found end time:   " << retval.end_pose.utime << std::endl;
    }

    //lasers_.pop();
    return retval;
}

void ApproxLaser::addPose(const maebot_pose_t* newPose)
{
    while(poses_.size() >= 5)
    {
        poses_.pop_front();
    }
    
    poses_.push_back(*newPose);
}
