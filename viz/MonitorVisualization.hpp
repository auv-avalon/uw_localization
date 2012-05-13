#ifndef UW_PARTICLE_LOCALIZATION_MONITORVIZ_HPP
#define UW_PARTICLE_LOCALIZATION_MONITORVIZ_HPP

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <uw_localization/types/environment.hpp>
#include <uw_localization/types/particle.hpp>
#include <uw_localization/types/info.hpp>

namespace vizkit {

class MonitorVisualization
    : public vizkit::Vizkit3DPlugin<uw_localization::Environment>,
      public vizkit::VizPluginAddType<uw_localization::ParticleSet>,
      public vizkit::VizPluginAddType<uw_localization::ParticleInfo>,
    boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(bool dynamic_map READ getDynamicMap WRITE setDynamicMap)
    Q_PROPERTY(bool gridlines READ getGridlines WRITE setGridlines)
    Q_PROPERTY(bool box READ getBox WRITE setBox)
    Q_PROPERTY(bool sonar_desire_point READ getDesirePoint WRITE setDesirePoint)
    Q_PROPERTY(bool sonar_real_point READ getRealPoint WRITE setRealPoint)

 public:
     MonitorVisualization() {
        VizPluginRubyAdapter(MonitorVisualization, uw_localization::Environment,  Environment);
        VizPluginRubyAdapter(MonitorVisualization, uw_localization::ParticleSet,  ParticleSet);
        VizPluginRubyAdapter(MonitorVisualization, uw_localization::ParticleInfo, ParticleInfo);
        map_created = false;
        map_changed = false;
        particle_update = false;
        sonar_update = false;
        property_dynamic_map = false;
        property_box = false;
        property_real_point = true;
        property_desire_point = true;
        property_gridlines = false;
     }

     ~MonitorVisualization();

     bool getDynamicMap() const { return property_dynamic_map; }
     bool getBox() const { return property_box; }
     bool getRealPoint() const { return property_real_point; }
     bool getDesirePoint() const { return property_desire_point; }
     bool getGridlines() const { return property_gridlines; }

     void setGridlines(bool p) { property_gridlines = p; }
     void setBox(bool p) { property_box = p; }
     void setDynamicMap(bool p) { property_dynamic_map = p; }
     void setRealPoint(bool p) { property_real_point = p; }
     void setDesirePoint(bool p) { property_desire_point = p; }

 protected:
     virtual osg::ref_ptr<osg::Node> createMainNode();
     virtual void updateMainNode(osg::Node* node);
     virtual void updateDataIntern(const uw_localization::Environment& env);
     virtual void updateDataIntern(const uw_localization::ParticleSet& set);
     virtual void updateDataIntern(const uw_localization::ParticleInfo& p);

     void renderEnvironment(const uw_localization::Environment& env);
     void renderParticles();
     void renderStatus();

     virtual osg::ref_ptr<osg::Geode> createPlaneNode(const uw_localization::Plane& plane);

 private:
     osg::ref_ptr<osg::Vec3Array> border_points;
     osg::ref_ptr<osg::Vec4Array> border_colors;
     osg::ref_ptr<osg::Geometry>  border_geom;
     osg::ref_ptr<osg::DrawArrays> grid;

     bool map_created;
     bool map_changed;
     bool particle_update;
     bool sonar_update;

     osg::ref_ptr<osg::Group> plane_group;
     osg::ref_ptr<osg::Group> particle_group;

     bool property_dynamic_map;
     bool property_box;
     bool property_real_point;
     bool property_desire_point;
     bool property_gridlines;

     double max_weight;
     unsigned int best_particle;

     uw_localization::Environment data_env;
};

}

#endif 
