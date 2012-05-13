#ifndef UW_LOCALIZATION_VIZ_PARTICLE_GEODE_HPP
#define UW_LOCALIZATION_VIZ_PARTICLE_GEODE_HPP

#include <osg/Geode>
#include <osg/Geometry>
#include <uw_localization/types/particle.hpp>
#include <uw_localization/types/info.hpp>

namespace vizkit {


class ParticleGeode : public osg::Geode
{
 public:
     enum VizType {
         BOX, LINE
     };

     ParticleGeode();
     virtual ~ParticleGeode();

     void renderAsBox(const uw_localization::Particle& particle, double size, double scale = 1.0);
     void renderAsLine(const uw_localization::Particle& particle, double from, double height, double size, double scale = 1.0);

     void renderRealPoint();
     void renderDesirePoint();

     void render();

     void updateParticle(const uw_localization::Particle& particle);
     void updateSonar(const uw_localization::PointInfo& info);
     void showRealPoint(bool show);
     void showDesirePoint(bool show);

     static osg::Vec4& gradient(double weight);
     static void setViz(VizType type, double min_z, double max_z, double size, double scale);

 private:
     // particle
     osg::ref_ptr<osg::Geometry> geom;
     osg::ref_ptr<osg::Vec4Array> color;
     osg::ref_ptr<osg::Vec3Array> vertices;

     osg::ref_ptr<osg::Geometry> sonar_geom;
     osg::ref_ptr<osg::Vec4Array> sonar_color;
     osg::ref_ptr<osg::Vec3Array> sonar_vertices;
     osg::ref_ptr<osg::DrawArrays> sonar_draw;

     osg::ref_ptr<osg::Geometry> desire_geom;
     osg::ref_ptr<osg::Vec4Array> desire_color;
     osg::ref_ptr<osg::Vec3Array> desire_vertices;
     osg::ref_ptr<osg::DrawArrays> desire_draw;

     static std::vector<osg::Vec4> color_map;

     uw_localization::PointInfo sonar;
     uw_localization::Particle particle;
     bool desire_point_show;
     bool real_point_show;
     bool changed;
     
     static VizType type;
     static double min_z;
     static double max_z;
     static double size;
     static double scale;
};


}

#endif
