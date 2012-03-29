#include "ParticleVisualization.hpp"
#include <osg/Geometry>

#define DATA(attribute) (p->attribute)

#define BLUE p->color_map.at(256)
#define GRADIENT(weight) p->color_map.at(floor(weight * 255))

namespace vizkit {

struct ParticleVisualization::Data {
    uw_localization::ParticleSet particleSet;
    std::vector<osg::Vec4> color_map;
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec3Array> points;
    osg::ref_ptr<osg::Vec4Array> colors;
    osg::ref_ptr<osg::DrawArrays> draw;

    double max_particle_height;
    bool show_highlight;
    bool show_scaling;
    bool show_colors;
};


ParticleVisualization::ParticleVisualization()
    : p (new Data)
{
    VizPluginRubyAdapter(ParticleVisualization, uw_localization::ParticleSet, Particles);

    for(unsigned i = 0; i < 256; i++) {
        if(i < 128)
            DATA(color_map).push_back(osg::Vec4(1.0f, i / 127.0f, 0.0f, 1.0f));
        else
            DATA(color_map).push_back(osg::Vec4(1.0f - ((i - 128) / 127.0f), 1.0f, 0.0f, 1.0f));
    }

   DATA(color_map).push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));

   p->max_particle_height = 2.0;
   p->show_highlight = true;
   p->show_scaling = true;
   p->show_colors = true;
}

ParticleVisualization::~ParticleVisualization()
{
    delete p;
}



double ParticleVisualization::getHeight() const
{
    return DATA(max_particle_height);
}


void ParticleVisualization::setHeight(double height)
{
    DATA(max_particle_height) = abs(height);
}


bool ParticleVisualization::isHighlight() const
{
    return DATA(show_highlight);
}


void ParticleVisualization::setHighlight(bool highlight)
{
    DATA(show_highlight) = highlight;
}


bool ParticleVisualization::isScaling() const
{
    return DATA(show_scaling);
}


void ParticleVisualization::setScaling(bool s)
{
    DATA(show_scaling) = s;
}


bool ParticleVisualization::isColor() const
{
    return DATA(show_colors);
}


void ParticleVisualization::setColor(bool colors)
{
    DATA(show_colors) = colors;
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
    std::vector<uw_localization::Particle>::const_iterator it;

    const uw_localization::ParticleSet& set = DATA(particleSet);

    double scaling = set.particles[set.max_particle_index].norm_weight;
    unsigned index = 0;

    for(it = set.particles.begin(); it != set.particles.end(); it++) {
        double weight = it->norm_weight / scaling;
        osg::Vec3d vec(it->position(0), it->position(1), 0.0);
        DATA(points)->push_back(vec);

        if(DATA(show_scaling))
            DATA(points)->push_back(vec + osg::Vec3d(0.0, 0.0, weight * DATA(max_particle_height)));
        else
            DATA(points)->push_back(vec + osg::Vec3d(0.0, 0.0, DATA(max_particle_height)));
        
        if(DATA(show_highlight) && index == set.max_particle_index)
            DATA(colors)->push_back(BLUE);
        else if(DATA(show_colors)) {
            DATA(colors)->push_back(GRADIENT(weight));
        } else
            DATA(colors)->push_back(GRADIENT(0.5));

        index++;
    }

    DATA(draw)->setCount(DATA(points)->size());
    DATA(geom)->setVertexArray(DATA(points));
    DATA(geom)->setColorArray(DATA(colors));
}


void
ParticleVisualization::updateDataIntern(uw_localization::ParticleSet const& value)
{
    DATA(particleSet) = value;
}
 
VizkitQtPlugin(ParticleVisualization)
} // namespace vizkit
