#ifndef UW_LOCALIZATION_MAPFEATUREVISUALIZATION_HPP
#define UW_LOCALIZATION_MAPFEATUREVISUALIZATION_HPP

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <vector>
#include <osg/Geode>
#include <uw_localization/types/map.hpp>

namespace vizkit {

class MapFeatureVisualization 
    : public vizkit::Vizkit3DPlugin<std::vector<uw_localization::Linemark> >,
      public vizkit::VizPluginAddType< std::vector<uw_localization::Landmark> >
    , boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(double height READ getHeight WRITE setHeight)

public:
    MapFeatureVisualization();
    ~MapFeatureVisualization();

    void setHeight(double height) { this->height = height; }
    double getHeight() const { return this->height; }

protected:
    virtual osg::ref_ptr<osg::Node> createMainNode();
    virtual void updateMainNode(osg::Node* node);
    virtual void updateDataIntern(std::vector<uw_localization::Linemark> const& lines);
    virtual void updateDataIntern(std::vector<uw_localization::Landmark> const& marks);

private:
    std::vector<uw_localization::Linemark> lines;
    std::vector<uw_localization::Landmark> marks;

    osg::ref_ptr<osg::Geometry> geom;
    osg::ref_ptr<osg::Vec3Array> vertices;
    osg::ref_ptr<osg::Vec4Array> colors;
    double height;
    bool line_changed;
    bool mark_changed;
};
}

#endif
