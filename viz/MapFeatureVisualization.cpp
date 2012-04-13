#include "MapFeatureVisualization.hpp"
#include <vizkit/Uncertainty.hpp>
#include <osg/Geometry>
#include <iostream>

namespace vizkit {

MapFeatureVisualization::MapFeatureVisualization() : height(0.5)
{
    VizPluginRubyAdapter(MapFeatureVisualization, uw_localization::MixedMap, Map);
    line_changed = true;
    mark_changed = true;
}


MapFeatureVisualization::~MapFeatureVisualization()
{
}


osg::ref_ptr<osg::Node> MapFeatureVisualization::createMainNode() {
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


void MapFeatureVisualization::updateMainNode(osg::Node* node )
{
    if(mark_changed) {
        osg::Group* group = node->asGroup();
        osg::Group* landmarks = group->getChild(0)->asGroup();

        // visualize landmarks
        if(marks.size() < landmarks->getNumChildren())
            landmarks->removeChildren(0, landmarks->getNumChildren() - marks.size());

        while(marks.size() > landmarks->getNumChildren()) {
            landmarks->addChild(new Uncertainty);
        }

        for(unsigned i = 0; i < marks.size(); i++) {
            Uncertainty* uncertainty = dynamic_cast<Uncertainty*>(landmarks->getChild(i));  
            uncertainty->hideSamples();
            uncertainty->setMean(static_cast<Eigen::Vector3d>(marks[i].mean));
            uncertainty->setCovariance(static_cast<Eigen::Matrix3d>(marks[i].covariance));
        }

        mark_changed = false;
    }

    if(line_changed) {
	vertices->clear();
	for(unsigned i = 0; i < lines.size(); i++) {
	    const uw_localization::Linemark& line = lines[i];
	    
            osg::Vec3d vec1(line.from.x(), line.from.y(), line.from.z());
	    osg::Vec3d vec2(line.to.x(), line.to.y(), line.from.z());
	
	    vertices->push_back(vec1 + osg::Vec3d(0.0, 0.0, -height * 0.5));
	    vertices->push_back(vec1 + osg::Vec3d(0.0, 0.0, height * 0.5));
            vertices->push_back(vec2 + osg::Vec3d(0.0, 0.0, height * 0.5));
	    vertices->push_back(vec2 + osg::Vec3d(0.0, 0.0, -height * 0.5));
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

        line_changed = false;
    }
}

void
MapFeatureVisualization::updateDataIntern(std::vector<uw_localization::Linemark> const& value)
{
    lines = value;
    line_changed = true;
}
 
void
MapFeatureVisualization::updateDataIntern(std::vector<uw_localization::Landmark> const& value)
{
    marks = value;
    mark_changed = true;
}
 
VizkitQtPlugin(MapFeatureVisualization)
}
