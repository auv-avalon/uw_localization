#ifndef UW_LOCALIZATION_VIZ_PARTICLE_GEODE_HPP
#define UW_LOCALIZATION_VIZ_PARTICLE_GEODE_HPP

#include <osg/Geode>
#include <osg/Geometry>
#include <uw_localization/types/particle.hpp>

namespace vizkit {


class ParticleGeode : public osg::Geode
{
 public:
     ParticleGeode();
     virtual ~ParticleGeode();

     void update_as_box(const uw_localization::Particle& particle, double size, double scale = 1.0);
     void update_as_line(const uw_localization::Particle& particle, double from, double height, double size, double scale = 1.0);

     static osg::Vec4& gradient(double weight);
 private:
     osg::ref_ptr<osg::Geometry> geom;
     osg::ref_ptr<osg::Vec4Array> color;
     osg::ref_ptr<osg::Vec3Array> vertices;

     static std::vector<osg::Vec4> color_map;
};


}

#endif
