#ifndef BOXVISUALIZATION_HPP
#define BOXVISUALIZATION_HPP

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <laser_filter/BoxFilter.hpp>
#include <base/eigen.h>

namespace osg {
    class Geometry;
}

namespace vizkit3d {

class BoxVisualization: public Vizkit3DPlugin<laser_filter::Box>
{
private:
    osg::ref_ptr<osg::Geode> boxNode;
    osg::ref_ptr<osg::Geometry> boxGeom;
    laser_filter::Box boxData;

public:
    BoxVisualization();
    
    virtual void updateDataIntern(const laser_filter::Box& data);    
    virtual void updateMainNode(osg::Node* node);
    virtual osg::ref_ptr< osg::Node > createMainNode();
};

}
#endif // BOXVISUALIZATION_HPP
