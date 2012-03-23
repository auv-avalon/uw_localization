#ifndef eras__ParticleVisualization_H
#define eras__ParticleVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <uw_localization/types/particle.hpp>

namespace vizkit {

class ParticleVisualization 
    : public vizkit::Vizkit3DPlugin< std::vector<uw_localization::Particle> >
    , boost::noncopyable    
{
    Q_OBJECT

    public:
        ParticleVisualization();
        ~ParticleVisualization();

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode ();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(std::vector<uw_localization::Particle> const& particles);

    private:
        struct Data;
        Data* p;
};

}

#endif
