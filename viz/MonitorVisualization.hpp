#ifndef UW_PARTICLE_LOCALIZATION_MONITORVIZ_HPP
#define UW_PARTICLE_LOCALIZATION_MONITORVIZ_HPP

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <uw_localization/types/environment.hpp>

namespace vizkit {

class MonitorVisualization
    : public vizkit::Vizkit3DPlugin<uw_localization::Environment>,
    boost::noncopyable
{
    Q_OBJECT

 public:
     MonitorVisualization();
     ~MonitorVisualization();

 protected:
     virtual osg::ref_ptr<osg::Node> createMainNode();
     virtual void updateMainNode(osg::Node* node);
     virtual void updateDataIntern(const uw_localization::Environment& env);

 private:
     osg::ref_ptr<osg::Vec3Array> border_points;
     osg::ref_ptr<osg::Vec4Array> border_colors;
     osg::ref_ptr<osg::Geometry>  border_geom;
     osg::ref_ptr<osg::DrawArrays> grid;
};

}

#endif 
