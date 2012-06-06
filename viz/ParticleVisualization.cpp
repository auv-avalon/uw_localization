#include "ParticleVisualization.hpp"
#include "ParticleGeode.hpp"
#include <osg/Geometry>
#include <boost/assert.hpp>

namespace vizkit {

ParticleVisualization::ParticleVisualization()
{
    VizPluginRubyAdapter(ParticleVisualization, uw_localization::ParticleSet, Particles);

    property_box = false;
    updated = false;
    max_weight = 1.0;

    property_min_z = 0.0;
    property_max_z = 1.0;
}

ParticleVisualization::~ParticleVisualization()
{}

osg::ref_ptr<osg::Node>
ParticleVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> root = new osg::Group;
    particle_group = new osg::Group;

    root->addChild(particle_group.get());

    return root;
}


void
ParticleVisualization::updateMainNode( osg::Node* node )
{
   if(updated)
        renderParticles();

   updated = false;
}


void ParticleVisualization::renderParticles()
{
    /*
    double max_z = data_env.right_bottom_corner.z();
    double min_z = data_env.left_top_corner.z();
    */

    if(property_box)
        ParticleGeode::setViz(ParticleGeode::BOX, property_max_z, property_min_z, 0.1, max_weight);
    else
        ParticleGeode::setViz(ParticleGeode::LINE, property_max_z, property_min_z, 0.1, max_weight);

    for(unsigned i = 0; i < particle_group->getNumChildren(); i++) {
        dynamic_cast<ParticleGeode*>(particle_group->getChild(i))->render();
    }
}


void
ParticleVisualization::updateDataIntern(uw_localization::ParticleSet const& p)
{
    while(particle_group->getNumChildren() < p.particles.size()) {
        osg::ref_ptr<ParticleGeode> geode = new ParticleGeode;
        particle_group->addChild(geode.get());
    }

    max_weight = p.particles[p.best_particle].main_confidence;

    for(unsigned i = 0; i < particle_group->getNumChildren(); i++) 
        dynamic_cast<ParticleGeode*>(particle_group->getChild(i))->updateParticle(p.particles[i]);

    updated = true;
}
 
VizkitQtPlugin(ParticleVisualization)
} // namespace vizkit
