#include "ParticleVisualization.hpp"
#include "ParticleGeode.hpp"
#include <algorithm>
#include <osg/Geometry>
#include <boost/assert.hpp>

namespace vizkit3d {

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


void ParticleVisualization::updateMainNode( osg::Node* node )
{  
   std::cout << "UpdateMainNode" << std::endl;
   if(updated)
        renderParticles();

   updated = false;
}


void ParticleVisualization::renderParticles()
{
    if(property_box)
        ParticleGeode::setViz(ParticleGeode::BOX, property_max_z, property_min_z, 0.1, max_weight);
    else
        ParticleGeode::setViz(ParticleGeode::LINE, property_max_z, property_min_z, 0.1, max_weight);

    for(unsigned i = 0; i < particle_group->getNumChildren(); i++) {
        dynamic_cast<ParticleGeode*>(particle_group->getChild(i))->render();
    }
}

void ParticleVisualization::updateDataIntern(uw_localization::ParticleSet const& p)
{
  particles = p;
  updated = true;
}


/*
void ParticleVisualization::updateDataIntern(uw_localization::ParticleSet const& p)
{    
    std::cout << "UpdateDataIntern" << std::endl;
    while(particle_group->getNumChildren() < p.particles.size()) {
        osg::ref_ptr<ParticleGeode> geode = new ParticleGeode;
        particle_group->addChild(geode.get());
    }

    max_weight = 0.0;

    for(unsigned j = 0; j < p.particles.size(); j++)
        max_weight = p.particles[j].main_confidence > max_weight 
            ? p.particles[j].main_confidence 
            : max_weight;

    for(unsigned i = 0; i < particle_group->getNumChildren(); i++) 
        dynamic_cast<ParticleGeode*>(particle_group->getChild(i))->updateParticle(p.particles[i]);

    updated = true;
    std::cout << "UpdateDataInternEnd" << std::endl;
}*/
 
//VizkitQtPlugin(ParticleVisualization)
} // namespace vizkit
