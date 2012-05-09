#ifndef UW_PARTICLE_LOCALIZATION_MONITORVIZ_HPP
#define UW_PARTICLE_LOCALIZATION_MONITORVIZ_HPP

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <uw_localization/types/environment.hpp>
#include <uw_localization/types/particle.hpp>

namespace vizkit {

class MonitorVisualization
    : public vizkit::Vizkit3DPlugin<uw_localization::Environment>,
      public vizkit::VizPluginAddType<uw_localization::ParticleSet>,
    boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(bool dynamic_map READ getDynamicMap WRITE setDynamicMap)
    Q_PROPERTY(bool box READ getBox WRITE setBox)

 public:
     MonitorVisualization() {
        VizPluginRubyAdapter(MonitorVisualization, uw_localization::Environment, Environment);
        VizPluginRubyAdapter(MonitorVisualization, uw_localization::ParticleSet, ParticleSet);
        map_created = false;
        particle_update = false;
        property_dynamic_map = false;
        property_box = false;
     }

     ~MonitorVisualization();

     bool getDynamicMap() const { return property_dynamic_map; }
     bool getBox() const { return property_box; }

     void setBox(bool p) { property_box = p; }
     void setDynamicMap(bool p) { property_dynamic_map = p; }

 protected:
     virtual osg::ref_ptr<osg::Node> createMainNode();
     virtual void updateMainNode(osg::Node* node);
     virtual void updateDataIntern(const uw_localization::Environment& env);
     virtual void updateDataIntern(const uw_localization::ParticleSet& set);

     void renderEnvironment(const uw_localization::Environment& env);
     void renderParticles(const uw_localization::ParticleSet& particles);
     void renderStatus();

     virtual osg::ref_ptr<osg::Geode> createPlaneNode(const uw_localization::Plane& plane);

 private:
     osg::ref_ptr<osg::Vec3Array> border_points;
     osg::ref_ptr<osg::Vec4Array> border_colors;
     osg::ref_ptr<osg::Geometry>  border_geom;
     osg::ref_ptr<osg::DrawArrays> grid;
     bool map_created;
     bool particle_update;

     osg::ref_ptr<osg::Group> plane_group;
     osg::ref_ptr<osg::Group> particle_group;

     bool property_dynamic_map;
     bool property_box;

     uw_localization::Environment data_env;
     uw_localization::ParticleSet data_particles;
};

}

#endif 
