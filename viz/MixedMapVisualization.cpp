#include "MixedMapVisualization.hpp"
#include <vizkit/Uncertainty.hpp>
#include <osg/Geometry>
#include <iostream>

namespace vizkit {

MixedMapVisualization::MixedMapVisualization()
{
    VizPluginRubyAdapter(MixedMapVisualization, uw_localization::MixedMap, Map);
    changed = true;
}


MixedMapVisualization::~MixedMapVisualization()
{
}


osg::ref_ptr<osg::Node> MixedMapVisualization::createMainNode() {
    osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<osg::Group> world = new osg::Group;
    osg::ref_ptr<osg::Group> landmarks = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    root->addChild(world);
    root->addChild(landmarks);

    world->addChild(geode);

    return root;
}


void MixedMapVisualization::updateMainNode(osg::Node* node )
{
    if(changed) {
        Eigen::Translation3d tr = Eigen::Translation3d(map.translation);
        Eigen::Vector3d lt = map.limitations;

        osg::Group* group = node->asGroup();
        osg::Group* landmarks = group->getChild(1)->asGroup();

        // visualize landmarks
        if(map.landmarks.size() < landmarks->getNumChildren())
            landmarks->removeChildren(0, landmarks->getNumChildren() - map.landmarks.size());

        while(map.landmarks.size() > landmarks->getNumChildren()) {
            landmarks->addChild(new Uncertainty);
        }

        for(unsigned i = 0; i < map.landmarks.size(); i++) {
            Uncertainty* uncertainty = dynamic_cast<Uncertainty*>(landmarks->getChild(i));  
            uncertainty->hideSamples();
            uncertainty->setMean(static_cast<Eigen::Vector3d>(map.landmarks[i].mean + base::Vector3d(0.0, 0.0, lt.z())));
            uncertainty->setCovariance(static_cast<Eigen::Matrix3d>(map.landmarks[i].covariance));
        }

        changed = false;
    }
}

void
MixedMapVisualization::updateDataIntern(uw_localization::MixedMap const& value)
{
    map = value;
    changed = true;
}
 
VizkitQtPlugin(MixedMapVisualization)
}
