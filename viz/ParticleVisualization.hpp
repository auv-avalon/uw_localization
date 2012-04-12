#ifndef eras__ParticleVisualization_H
#define eras__ParticleVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <uw_localization/types/particle.hpp>

namespace vizkit {

class ParticleVisualization 
    : public vizkit::Vizkit3DPlugin<uw_localization::debug::ParticleSet>
    , boost::noncopyable    
{
    Q_OBJECT
    Q_PROPERTY(double max_height READ getHeight WRITE setHeight)
    Q_PROPERTY(bool scaling READ isScaling WRITE setScaling)
    Q_PROPERTY(bool colors READ isColor WRITE setColor)

    public:
        ParticleVisualization();
        ~ParticleVisualization();

        double getHeight() const;
        void   setHeight(double height);

        bool isScaling() const;
        void setScaling(bool);

        bool isColor() const;
        void setColor(bool colors);


    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode ();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(uw_localization::debug::ParticleSet const& set);

    private:
        struct Data;
        Data* p;
};

}

#endif
