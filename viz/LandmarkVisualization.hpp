#ifndef UW_LOCALIZATION_LANDMARKVISUALIZATION_HPP
#define UW_LOCALIZATION_LANDMARKVISUALIZATION_HPP

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <uw_localization/types/map.hpp>

namespace vizkit {

class LandmarkVisualization 
    : public vizkit::Vizkit3DPlugin<uw_localization::LandmarkMap>
    , boost::noncopyable
{
    Q_OBJECT
public:
    LandmarkVisualization();
    ~LandmarkVisualization();

protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(uw_localization::LandmarkMap const& map);

private:
    uw_localization::LandmarkMap map;
    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec4Array> colors;
    bool changed;
};
}

#endif
