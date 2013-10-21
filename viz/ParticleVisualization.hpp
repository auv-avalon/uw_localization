#ifndef eras__ParticleVisualization_H
#define eras__ParticleVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Group>
#include <uw_localization/types/particle.hpp>

namespace vizkit3d {

class ParticleVisualization 
    : public vizkit3d::Vizkit3DPlugin<uw_localization::ParticleSet>
    , boost::noncopyable    
{
    Q_OBJECT
    Q_PROPERTY(bool box READ getBox WRITE setBox)
    Q_PROPERTY(double max_z READ getMaxZ WRITE setMaxZ)
    Q_PROPERTY(double min_z READ getMinZ WRITE setMinZ)

    public:
        ParticleVisualization();
        ~ParticleVisualization();

        bool getBox() const { return property_box; }
        void setBox(bool p) { property_box = p; }

        double getMaxZ() const { return property_max_z; }
        void   setMaxZ(double z) { property_max_z = z; }

        double getMinZ() const { return property_min_z; }
        void   setMinZ(double z) { property_min_z = z; }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode ();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(uw_localization::ParticleSet const& set);
        void renderParticles();

    private:
        double property_max_z;
        double property_min_z;
        bool property_box;
        bool updated;
        double max_weight;
       
        osg::ref_ptr<osg::Group> particle_group;
};

}

#endif
