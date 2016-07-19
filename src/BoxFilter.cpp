#include "BoxFilter.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

namespace laser_filter {
    
BoxFilter::BoxFilter(): filterFrame(Eigen::Affine3d::Identity())
{

}

void BoxFilter::addBoundingBox(const laser_filter::Box& box)
{
    // upRight actually means Max and downLeft Min.
    base::Vector3d max_vec;
    max_vec[0] = std::max(box.downLeft[0], box.upRight[0]);
    max_vec[1] = std::max(box.downLeft[1], box.upRight[1]);
    max_vec[2] = std::max(box.downLeft[2], box.upRight[2]);
    base::Vector3d min_vec;
    min_vec[0] = std::min(box.downLeft[0], box.upRight[0]);
    min_vec[1] = std::min(box.downLeft[1], box.upRight[1]);
    min_vec[2] = std::min(box.downLeft[2], box.upRight[2]);
    boundingBoxes.push_back(Eigen::AlignedBox<double, 3>(min_vec, max_vec));
}

void BoxFilter::setFilterFrame(const Eigen::Affine3d& ff)
{
    filterFrame = ff;
}
    
void BoxFilter::filter(base::samples::LaserScan& filterdScan, const base::samples::LaserScan& ls)
{
    if(boundingBoxes.empty())
    {
        filterdScan = ls;
        return;
    }
    
    std::vector<Eigen::Vector3d> pointCloud;

    //copy attributes to filtered scan
    filterdScan = ls;

    for(unsigned int i = 0; i < ls.ranges.size(); i++) {

	Eigen::Vector3d curPoint;
	
	//convert reading to cartesian coordinates
	if(!ls.getPointFromScanBeamXForward(i, curPoint))
	    continue;
	
	//transform into filter frame
	curPoint = filterFrame * curPoint;

	//check for intersection with masked areas
	for(std::vector<Eigen::AlignedBox<double, 3> >::const_iterator it = boundingBoxes.begin(); it != boundingBoxes.end();it++)
	{
	    if(it->contains(curPoint))
	    {
		//as we don't have a better error this is an other range error for now
		filterdScan.ranges[i] = base::samples::OTHER_RANGE_ERRORS;
// 		std::cout << "Point in masked Area " << curPoint.transpose() << std::endl;
		break;
	    }
	}
    }
}

}
