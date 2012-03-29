#include "LandmarkVisualization.hpp"
#include <vizkit/Uncertainty.hpp>
#include <osg/Geometry>

namespace vizkit {

LandmarkVisualization::LandmarkVisualization()
{
    VizPluginRubyAdapter(LandmarkVisualization, uw_localization::LandmarkMap, Map);
}


LandmarkVisualization::~LandmarkVisualization()
{
}


osg::ref_ptr<osg::Node> LandmarkVisualization::createMainNode() {
    return new osg::Group;
}


void LandmarkVisualization::updateMainNode(osg::Node* node )
{
    osg::Group* group = node->asGroup();

    while(map.landmarks.size() != group->getNumChildren()) {
        if(map.landmarks.size() < group->getNumChildren())
            group->removeChildren(0, group->getNumChildren() - map.landmarks.size());

        if(map.landmarks.size() > group->getNumChildren())
            group->addChild(new Uncertainty);
    }

    for(unsigned i = 0; i < map.landmarks.size(); i++) {
        Uncertainty* uncertainty = dynamic_cast<Uncertainty*>(group->getChild(i));  
        uncertainty->hideSamples();
        uncertainty->setMean(static_cast<Eigen::Vector3d>(map.landmarks[i].mean));
        uncertainty->setCovariance(static_cast<Eigen::Matrix3d>(map.landmarks[i].covariance));
    }
}

void
LandmarkVisualization::updateDataIntern(uw_localization::LandmarkMap const& value)
{
    map = value;
}
 
VizkitQtPlugin(LandmarkVisualization)
}
