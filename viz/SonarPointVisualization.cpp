#include "SonarPointVisualization.hpp"

namespace vizkit {

osg::ref_ptr<osg::Node> SonarPointVisualization::createMainNode()
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;

    sonar_color = new osg::Vec4Array;
    sonar_vertices = new osg::Vec3Array;
    sonar_geom = new osg::Geometry;
    sonar_draw = new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4);

    desire_color = new osg::Vec4Array;
    desire_vertices = new osg::Vec3Array;
    desire_geom = new osg::Geometry;
    desire_draw = new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 8);

    sonar_geom->addPrimitiveSet(sonar_draw.get());
    sonar_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    sonar_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    desire_geom->addPrimitiveSet(desire_draw.get());
    desire_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    desire_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    geode->addDrawable(desire_geom.get());
    geode->addDrawable(sonar_geom.get());

    return geode;
}


void SonarPointVisualization::updateMainNode(osg::Node* node)
{
    if(updated) {
        desire_color->clear();
        desire_vertices->clear();
        sonar_color->clear();
        sonar_vertices->clear();

        osg::Vec3 pos(point.location.x(), point.location.y(), point.location.z());
        osg::Vec3 des(point.desire_point.x(), point.desire_point.y(), 0.0f);
        osg::Vec3 son(point.real_point.x(), point.real_point.y(), 0.0f);

        desire_vertices->push_back(des + osg::Vec3(-0.1f, 0.0f, property_min_z));
        desire_vertices->push_back(des + osg::Vec3(0.1f, 0.0f, property_min_z));
        desire_vertices->push_back(des + osg::Vec3(0.1f, 0.0f, property_max_z));
        desire_vertices->push_back(des + osg::Vec3(-0.1f, 0.0f, property_max_z));

        desire_vertices->push_back(des + osg::Vec3(0.0f, -0.1f, property_min_z));
        desire_vertices->push_back(des + osg::Vec3(0.0f, 0.1f, property_min_z));
        desire_vertices->push_back(des + osg::Vec3(0.0f, 0.1f, property_max_z));
        desire_vertices->push_back(des + osg::Vec3(0.0f, -0.1f, property_max_z));

        desire_color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 0.7f));
        desire_geom->setVertexArray(desire_vertices.get());
        desire_geom->setColorArray(desire_color.get());

        sonar_vertices->push_back(pos);
        sonar_vertices->push_back(pos);
        sonar_vertices->push_back(son + osg::Vec3(0.0f, 0.0f, property_min_z));
        sonar_vertices->push_back(son + osg::Vec3(0.0f, 0.0f, property_max_z));

        sonar_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.7f));
        sonar_geom->setVertexArray(sonar_vertices.get());
        sonar_geom->setColorArray(sonar_color.get());
    }

    updated = false;
}


void SonarPointVisualization::updateDataIntern(const uw_localization::PointInfo& point)
{
    if(point.status == uw_localization::OKAY) {
        this->point = point;
        updated = true;
    }
}


VizkitQtPlugin(SonarPointVisualization)
}
