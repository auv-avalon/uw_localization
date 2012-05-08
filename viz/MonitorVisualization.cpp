#include "MonitorVisualization.hpp"

namespace vizkit {

MonitorVisualization::MonitorVisualization()
{
    VizPluginRubyAdapter(MonitorVisualization, uw_localization::Environment, Environment);
}

MonitorVisualization::~MonitorVisualization()
{
}


osg::ref_ptr<osg::Node> MonitorVisualization::createMainNode()
{
    osg::ref_ptr<osg::Geode> border_geode(new osg::Geode);
    grid = new osg::DrawArrays(osg::PrimitiveSet::LINES, 18, 0);
    border_points = new osg::Vec3Array;
    border_colors = new osg::Vec4Array;
    border_geom = new osg::Geometry;
    border_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, 5));
    border_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 5, 5));
    border_geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 10, 8));
    border_geom->addPrimitiveSet(grid.get());

    border_geom->setColorBinding(osg::Geometry::BIND_OVERALL);

    border_geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    border_geode->addDrawable(border_geom.get());

    return border_geode;
}


void MonitorVisualization::updateMainNode(osg::Node* node)
{
}


void MonitorVisualization::updateDataIntern(const uw_localization::Environment& env)
{
    // render borders
    border_points->clear();

    const base::Vector3d& L = env.left_top_corner;
    const base::Vector3d& R = env.right_bottom_corner;

    // top side
    border_points->push_back(osg::Vec3d(L.x(), L.y(), L.z()));
    border_points->push_back(osg::Vec3d(L.x(), R.y(), L.z()));
    border_points->push_back(osg::Vec3d(R.x(), R.y(), L.z()));
    border_points->push_back(osg::Vec3d(R.x(), L.y(), L.z()));
    border_points->push_back(osg::Vec3d(L.x(), L.y(), L.z()));

    // bottom side
    border_points->push_back(osg::Vec3d(L.x(), L.y(), R.z()));
    border_points->push_back(osg::Vec3d(L.x(), R.y(), R.z()));
    border_points->push_back(osg::Vec3d(R.x(), R.y(), R.z()));
    border_points->push_back(osg::Vec3d(R.x(), L.y(), R.z()));
    border_points->push_back(osg::Vec3d(L.x(), L.y(), R.z()));

    // z borders
    border_points->push_back(osg::Vec3d(L.x(), L.y(), R.z()));
    border_points->push_back(osg::Vec3d(L.x(), L.y(), L.z()));

    border_points->push_back(osg::Vec3d(L.x(), R.y(), R.z()));
    border_points->push_back(osg::Vec3d(L.x(), R.y(), L.z()));

    border_points->push_back(osg::Vec3d(R.x(), L.y(), R.z()));
    border_points->push_back(osg::Vec3d(R.x(), L.y(), L.z()));

    border_points->push_back(osg::Vec3d(R.x(), R.y(), R.z()));
    border_points->push_back(osg::Vec3d(R.x(), R.y(), L.z()));

    // set border color to white
    while(border_colors->size() < border_points->size())
        border_colors->push_back(osg::Vec4d(1.0, 1.0, 1.0, 1.0));

    // draw grid lines
    double centre_x = (L.x() + R.x()) * 0.5;
    double centre_y = (L.y() + R.y()) * 0.5;

    double width  = (L.y() - R.y());
    double height = (L.x() - R.x());

    double dx = 0.0;
    double dy = 0.0;

    unsigned grid_vertices = 0;

    while( dy <= width * 0.5 ) {
        border_points->push_back(osg::Vec3d(centre_x - height / 2.0, centre_y + dy, R.z()));
        border_points->push_back(osg::Vec3d(centre_x + height / 2.0, centre_y + dy, R.z()));
        border_points->push_back(osg::Vec3d(centre_x - height / 2.0, centre_y - dy, R.z()));
        border_points->push_back(osg::Vec3d(centre_x + height / 2.0, centre_y - dy, R.z()));

        dy += 1.0;
        grid_vertices += 4;
    }

    while( dx <= height * 0.5 ) {
        border_points->push_back(osg::Vec3d(centre_x + dx, centre_y - width / 2.0, R.z()));
        border_points->push_back(osg::Vec3d(centre_x + dx, centre_y + width / 2.0, R.z()));
        
        border_points->push_back(osg::Vec3d(centre_x - dx, centre_y - width / 2.0, R.z()));
        border_points->push_back(osg::Vec3d(centre_x - dx, centre_y + width / 2.0, R.z()));
        
        dx += 1.0;
        grid_vertices += 4;
    }

    grid->setCount(grid_vertices);

    // set grid color to grey
    while(border_colors->size() < border_points->size())
        border_colors->push_back(osg::Vec4d(0.8, 0.7, 0.4, 0.5));

    border_geom->setColorArray(border_colors.get());
    border_geom->setVertexArray(border_points.get());
}

VizkitQtPlugin(MonitorVisualization)
}

