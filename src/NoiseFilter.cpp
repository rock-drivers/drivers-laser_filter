#include "NoiseFilter.hpp"

#include <math.h>
#include <base/samples/laser_scan.h>
#include <iostream>

namespace laser_filter {
    
NoiseFilter::NoiseFilter():minIncline(0.0), maxIncline(0.0), maskedNeighbours(0), maxRange(0.0)
{

}

double NoiseFilter::calculateInclineAngle(const double reading1, const double reading2, const double angleBetweenReadings) const
{
    return atan2(sin(angleBetweenReadings) * reading2, reading1 - (cos(angleBetweenReadings) * reading2));
}

void NoiseFilter::filter(base::samples::LaserScan& filterdScan, const base::samples::LaserScan& ls)
{
    std::vector<bool> maskedPoints;

    int lastRange = ls.ranges.at(0);
    int lastRangeIndex = 0;
    const int nrPoints = ls.ranges.size();    
    
    maskedPoints.resize(nrPoints);

    //copy attributes to filtered scan
    filterdScan = ls;

    // Set the maxRange for the filtered scan
    filterdScan.maxRange = maxRange*1000;

    for(unsigned int i = 0; i < maskedPoints.size(); i++) {
	maskedPoints[i] = false;
    }
    
//     std::cout << std::endl << std::endl << "NEW SCAN " << std::endl;
    
    for(unsigned int i = 0; i < ls.ranges.size(); i++) {

	if(!filterdScan.isRangeValid(ls.ranges[i]))
		maskedPoints[i] = true;

	
	const double incline = calculateInclineAngle(ls.ranges[i], lastRange, ls.angular_resolution * (i - lastRangeIndex));
	
// 	std::cout << "Range, " << ls.ranges[i] << " lastRange " << lastRange << " angle " << ls.angular_resolution * (i - lastRangeIndex) << " incline " << incline<< std::endl;
	
	//this is a filter for false readings that do occur if one scannes over edgeds of objects
	if(incline < minIncline || incline > maxIncline)
	{
	    //mask neighbour points
	    for(int j = -maskedNeighbours; j < maskedNeighbours; j++)
	    {
		if((int(i) +j < 0) 
		   || ((int(i)+j) > nrPoints))
		{
		    continue;
		}
		maskedPoints[i + j] = true;
	    }
	}
	
	lastRange = ls.ranges[i];
	lastRangeIndex = i;
    }

    //mark all masked points as invalid in scan 
    for(unsigned int i = 0; i < ls.ranges.size(); i++) {
	if(maskedPoints[i]) {
	    //as we don't have a better error this is an other range error for now
	    filterdScan.ranges[i] = base::samples::OTHER_RANGE_ERRORS;
	}	
    }    
}

}