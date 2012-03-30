#include "LandmarkVisualization.hpp"
#include <vizkit/Uncertainty.hpp>
#include <osg/Geometry>

namespace vizkit {

LandmarkVisualization::LandmarkVisualization()
{
    VizPluginRubyAdapter(LandmarkVisualization, uw_localization::LandmarkMap, Map);
    changed = true;
}


LandmarkVisualization::~LandmarkVisualization()
{
}


osg::ref_ptr<osg::Node> LandmarkVisualization::createMainNode() {
    osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<osg::Group> world = new osg::Group;
    osg::ref_ptr<osg::Group> landmarks = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    geom = new osg::Geometry;
    vertices = new osg::Vec3Array;
    colors = new osg::Vec4Array;

    colors->push_back(osg::Vec4(0.5f, 0.5f, 0.5f, 1.0f));
    geom->setVertexArray(vertices);
    geom->setColorArray(colors);
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    geode->addDrawable(geom.get());
    
    root->addChild(world);
    root->addChild(landmarks);

    world->addChild(geode);

    return root;
}


void LandmarkVisualization::updateMainNode(osg::Node* node )
{
    if(changed) {
        osg::Group* group = node->asGroup();
        osg::Group* landmarks = group->getChild(1)->asGroup();

        // visualize landmarks
        while(map.landmarks.size() != landmarks->getNumChildren()) {
            if(map.landmarks.size() < landmarks->getNumChildren())
                landmarks->removeChildren(0, landmarks->getNumChildren() - map.landmarks.size());

            if(map.landmarks.size() > landmarks->getNumChildren())
                group->addChild(new Uncertainty);
        }

        for(unsigned i = 0; i < map.landmarks.size(); i++) {
            Uncertainty* uncertainty = dynamic_cast<Uncertainty*>(landmarks->getChild(i));  
            uncertainty->hideSamples();
            uncertainty->setMean(static_cast<Eigen::Vector3d>(map.landmarks[i].mean));
            uncertainty->setCovariance(static_cast<Eigen::Matrix3d>(map.landmarks[i].covariance));
        }

        vertices->clear();

        // visualize world limitations#
        Eigen::Translation3d tr = Eigen::Translation3d(map.translation);
        Eigen::Vector3d lt = map.limitations;

        vertices->push_back(osg::Vec3d(tr.x(), tr.y(), tr.z()));
        vertices->push_back(osg::Vec3d(tr.x() + lt.x(), tr.y(), tr.z()));
        vertices->push_back(osg::Vec3d(tr.x() + lt.x(), tr.y(), tr.z() + lt.z()));
        vertices->push_back(osg::Vec3d(tr.x(), tr.y(), tr.z() + lt.z()));

        geom->setVertexArray(vertices);
        geom->removePrimitiveSet(0, geom->getNumPrimitiveSets());

        for(size_t i = 0; i < (vertices->size() - 3); i += 4) {
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
LandmarkVisualization::updateDataIntern(uw_localization::LandmarkMap const& value)
{
    map = value;
    changed = true;
}
 
VizkitQtPlugin(LandmarkVisualization)
}
