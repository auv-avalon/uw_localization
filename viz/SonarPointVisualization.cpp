#include "SonarPointVisualization.hpp"

namespace vizkit3d {

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

    hist_color = new osg::Vec4Array;
    hist_vertices = new osg::Vec3Array;
    hist_geom = new osg::Geometry;
    hist_draw = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 0);

    sonar_geom->addPrimitiveSet(sonar_draw.get());
    sonar_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    sonar_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    desire_geom->addPrimitiveSet(desire_draw.get());
    desire_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    desire_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    hist_geom->addPrimitiveSet(hist_draw.get());
    hist_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    hist_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);


    geode->addDrawable(desire_geom.get());
    geode->addDrawable(sonar_geom.get());
    geode->addDrawable(hist_geom.get());

    return geode;
}


void SonarPointVisualization::updateMainNode(osg::Node* node)
{
    if(updated) {
        desire_color->clear();
        desire_vertices->clear();
        sonar_color->clear();
        sonar_vertices->clear();
        hist_color->clear();
        hist_vertices->clear();

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

        desire_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.7f));
        desire_geom->setVertexArray(desire_vertices.get());
        desire_geom->setColorArray(desire_color.get());

        sonar_vertices->push_back(pos);
        sonar_vertices->push_back(pos);
        sonar_vertices->push_back(son + osg::Vec3(0.0f, 0.0f, property_min_z));
        sonar_vertices->push_back(son + osg::Vec3(0.0f, 0.0f, property_max_z));

        sonar_color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 0.7f));
        sonar_geom->setVertexArray(sonar_vertices.get());
        sonar_geom->setColorArray(sonar_color.get());

        std::list<uw_localization::PointInfo>::const_iterator it;
        for(it = history.begin(); it != history.end(); ++it)
        {
            osg::Vec3d vec(it->real_point.x(), it->real_point.y(), property_min_z);
            hist_vertices->push_back(vec);
            hist_vertices->push_back(vec + osg::Vec3d(0.0, 0.0, property_max_z));
        }

        hist_color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 0.7f));
        hist_geom->setColorArray(hist_color);

        if(hist_vertices->size() > 0) {
            hist_geom->setVertexArray(hist_vertices);
            dynamic_cast<osg::DrawArrays*>(hist_geom->getPrimitiveSet(0))->setCount(hist_vertices->size());
        }
    }

    updated = false;
}


void SonarPointVisualization::updateDataIntern(const uw_localization::PointInfo& point)
{
    if(point.status == uw_localization::OKAY) {
        this->history.push_back(this->point);
        
        while(this->history.size() > property_history)
            this->history.pop_front();

        this->point = point;
        updated = true;
    }
}


VizkitQtPlugin(SonarPointVisualization)
}
