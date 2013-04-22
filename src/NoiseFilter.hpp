#ifndef NOISEFILTER_HPP
#define NOISEFILTER_HPP

#include <base/samples/laser_scan.h>

namespace laser_filter {

class NoiseFilter
{
private:
    /**
    * This function calculates the inclination angle between the given laser readings.
    * 
    * */
    double calculateInclineAngle(const double reading1, const double reading2, const double angleBetweenReadings) const;
    
    /** 
     * Minimum allowed incline between to scan readings
     **/ 
    double minIncline;
    
    /** 
     * Maximum allowed incline between to scan readings
     **/ 
    double maxIncline;
    
    /**
     * Defines how many readings next to an invalid reading get marked as invalid
     * */
    int maskedNeighbours;
    double maxRange;
public:
    
    NoiseFilter();
    
    void setMinIncline(const double min) 
    {
	minIncline = min;
    }
    
    void setMaxIncline(const double max) 
    {
	maxIncline = max;
    }
    
    void setNumMaskedNeighbours(const int num)
    {
	maskedNeighbours = num;
    }
    
    void setMaxRange(const double max)
	{
	maxRange = max;
	}

    void filter(base::samples::LaserScan& filterdScan, const base::samples::LaserScan& ls_in);
};

}
#endif // NOISEFILTER_HPP
