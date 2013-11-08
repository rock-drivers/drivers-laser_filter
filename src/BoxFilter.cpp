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
    boundingBoxes.push_back(Eigen::AlignedBox<double, 3>(box.downLeft, box.upRight));
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