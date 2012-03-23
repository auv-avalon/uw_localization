#include "ParticleVisualization.hpp"

#include <osg/Geometry>

#define DATA(attribute) (p->attribute)

namespace vizkit {

struct ParticleVisualization::Data {
    std::vector<uw_localization::Particle> particles;
    std::vector<osg::Vec4> color_map;
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec3Array> points;
    osg::ref_ptr<osg::Vec4Array> colors;
    osg::ref_ptr<osg::DrawArrays> draw;
};


ParticleVisualization::ParticleVisualization()
    : p (new Data)
{
    VizPluginRubyAdapter(ParticleVisualization, std::vector<uw_localization::Particle>, Particles)

    DATA(color_map).push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
}

ParticleVisualization::~ParticleVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node>
ParticleVisualization::createMainNode()
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();

    DATA(geom) = new osg::Geometry;
    DATA(points) = new osg::Vec3Array;
    DATA(colors) = new osg::Vec4Array;
    DATA(draw) = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, DATA(points)->size());
    DATA(geom)->addPrimitiveSet(DATA(draw).get());
    
    DATA(colors)->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    DATA(geom)->setVertexArray(DATA(points));
    DATA(geom)->setColorArray(DATA(colors));
    DATA(geom)->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE);
    DATA(geom)->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    geode->addDrawable(DATA(geom).get());

    return geode;
}


void
ParticleVisualization::updateMainNode( osg::Node* node )
{
    DATA(points)->clear();
    DATA(colors)->clear();
    std::vector<uw_localization::Particle>::iterator it;

    for(it = DATA(particles).begin(); it != DATA(particles).end(); it++) {

        osg::Vec3d vec(it->position(0), it->position(1), 0.0);
        DATA(points)->push_back(vec);
        DATA(points)->push_back(vec + osg::Vec3d(0.0, 0.0, 0.25));
        DATA(colors)->push_back(DATA(color_map).at(0)); //static_cast<int>(it->confidence * 256)));
    }

    DATA(draw)->setCount(DATA(points)->size());
    DATA(geom)->setVertexArray(DATA(points));
    DATA(geom)->setColorArray(DATA(colors));
}


void
ParticleVisualization::updateDataIntern(std::vector<uw_localization::Particle> const& value)
{
    p->particles = value;
}
 
VizkitQtPlugin(ParticleVisualization)
} // namespace vizkit
