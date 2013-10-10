#include "BoxVisualization.hpp"
#include <osg/Geometry>
#include <osg/Geode>
#include <vizkit3d/Vizkit3DHelper.hpp>
#include <osg/ShapeDrawable>
#include <osg/Shape>

namespace vizkit3d {
VizkitQtPlugin(BoxVisualization)

using namespace Eigen;

BoxVisualization::BoxVisualization()
{
    VizPluginRubyAdapter(BoxVisualization, laser_filter::Box, Box)
}

osg::ref_ptr< osg::Node > BoxVisualization::createMainNode()
{
    boxNode = new osg::Geode();
    boxGeom = new osg::Geometry();

    //setup normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    boxGeom->setNormalArray(normals);
    boxGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
    
    //Set color
    osg::Vec4Array *colors = new osg::Vec4Array();
    colors->push_back(osg::Vec4(0,0.3,0,0.5));
    boxGeom->setColorArray(colors);
    boxGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    //turn on transparacny
    boxGeom->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    
    boxNode->addDrawable(boxGeom);

    return boxNode;
}

void BoxVisualization::updateDataIntern(const laser_filter::Box& data)
{
    boxData = data;
}

void BoxVisualization::updateMainNode(osg::Node* node)
{
    AlignedBox<double, 3> box(boxData.downLeft, boxData.upRight);
    
    Vector3d cp = box.center();
    Vector3d bs = box.sizes();
    
    if(boxNode->getNumDrawables())
	boxNode->removeDrawables(0);
    
    osg::Box *osgBox = new osg::Box(eigenVectorToOsgVec3(cp), bs.x(), bs.y(), bs.z());
    osg::ref_ptr<osg::ShapeDrawable> boxDrawable = new osg::ShapeDrawable(osgBox);
    boxDrawable->setColor(osg::Vec4d(0, 0.3, 0, 0.5));
    boxNode->addDrawable(boxDrawable);    
}


}
