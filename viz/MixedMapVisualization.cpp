#include "MixedMapVisualization.hpp"
#include <vizkit/Uncertainty.hpp>
#include <osg/Geometry>
#include <iostream>

namespace vizkit {

MixedMapVisualization::MixedMapVisualization() : height(0.5)
{
    VizPluginRubyAdapter(MixedMapVisualization, uw_localization::MixedMap, Map);
    changed = true;
}


MixedMapVisualization::~MixedMapVisualization()
{
}


osg::ref_ptr<osg::Node> MixedMapVisualization::createMainNode() {
    osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<osg::Group> landmarks = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    
    geom = new osg::Geometry;
    vertices = new osg::Vec3Array;
    colors = new osg::Vec4Array;
    
    geom->setVertexArray(vertices);
    colors->push_back(osg::Vec4(0.8f, 0.8f, 0.8f, 0.8f));
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    root->addChild(landmarks);
    geode->addDrawable(geom.get());

    root->addChild(geode);

    return root;
}


void MixedMapVisualization::updateMainNode(osg::Node* node )
{
    if(changed) {
        Eigen::Translation3d tr = Eigen::Translation3d(map.translation);
        Eigen::Vector3d lt = map.limitations;

        osg::Group* group = node->asGroup();
        osg::Group* landmarks = group->getChild(0)->asGroup();

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

	vertices->clear();
	for(unsigned i = 0; i < map.lines.size(); i++) {
	    const uw_localization::Linemark& line = map.lines[i];
	    
            osg::Vec3d vec1(line.from.x(), line.from.y(), 0.0);
	    osg::Vec3d vec2(line.to.x(), line.to.y(), 0.0);
	
	    vertices->push_back(vec1);
	    vertices->push_back(vec1 + osg::Vec3d(0.0, 0.0, height));
            vertices->push_back(vec2 + osg::Vec3d(0.0, 0.0, height));
	    vertices->push_back(vec2);
	}
	
	geom->removePrimitiveSet(0, geom->getNumPrimitiveSets());

	for(size_t i = 0; i < vertices->size() - 3; i += 4) {
            osg::ref_ptr<osg::DrawElementsUInt> draw = new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);

            draw->push_back(i);
            draw->push_back(i + 1);
            draw->push_back(i + 2);
            draw->push_back(i + 3);
            geom->addPrimitiveSet(draw);
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
