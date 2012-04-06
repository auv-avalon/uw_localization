#ifndef UW_LOCALIZATION_MIXEDMAPVISUALIZATION_HPP
#define UW_LOCALIZATION_MIXEDMAPVISUALIZATION_HPP

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <uw_localization/types/map.hpp>

namespace vizkit {

class MixedMapVisualization 
    : public vizkit::Vizkit3DPlugin<uw_localization::MixedMap>
    , boost::noncopyable
{
    Q_OBJECT
public:
    MixedMapVisualization();
    ~MixedMapVisualization();

protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(uw_localization::MixedMap const& map);

private:
    uw_localization::MixedMap map;
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec4Array> colors;
    bool changed;
};
}

#endif
