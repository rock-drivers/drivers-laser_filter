#ifndef BOXFILTER_HPP
#define BOXFILTER_HPP

#include <base/samples/LaserScan.hpp>
#include <base/Eigen.hpp>

namespace laser_filter {

/**
 * Transport (wrapper) class 
 * */
class Box {
public:
    base::Vector3d downLeft; // Min values
    base::Vector3d upRight; // Max values
};

/**
 * This Filter first transforms the laser scan into a point cloud using the
 * given transformation. Afterwards it checks if any of the points is inside
 * the given Boundingboxes. If Any point is inside of a BB it get's filtered 
 * from the laser scan
 * */
class BoxFilter
{
private:
    Eigen::Affine3d filterFrame;
    std::vector<Eigen::AlignedBox<double, 3> > boundingBoxes;
    
public:
    BoxFilter();
    
    /**
     * Set the frame to which the laser scan gets transformed.
     * */
    void setFilterFrame(const Eigen::Affine3d &ff);
    
    /**
     * Adds a Bounding Box for checking
     * */
    void addBoundingBox(const Box &box);
    
    /**
     * Performs the filtering
     * */
    void filter(base::samples::LaserScan& filterdScan, const base::samples::LaserScan& ls);
    
};

}
#endif // BOXFILTER_HPP
