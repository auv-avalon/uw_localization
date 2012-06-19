#include "MapVisualization.hpp"
#include <uw_localization/maps/node_map.hpp>

namespace vizkit {

MapVisualization::~MapVisualization()
{
}


void MapVisualization::updateDataIntern(const uw_localization::Environment& env)
{
    data_env = env;
    updated = true;
}


osg::ref_ptr<osg::Node> MapVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<osg::Geode> border_geode(new osg::Geode);
    plane_group = new osg::Group;

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
    
    root->addChild(border_geode.get());
    root->addChild(plane_group.get());

    setDirty();

    return root;
}


void MapVisualization::updateMainNode(osg::Node* node)
{
    if(updated || property_updated)
        renderEnvironment(data_env);
}

void MapVisualization::setMap(const QString& p) 
{
    uw_localization::NodeMap m(p.toAscii().data());

    std::cout << "Map: " << p.toAscii().data() << std::endl;

    data_env = m.getEnvironment();

    property_updated = true;
}


void MapVisualization::renderEnvironment(const uw_localization::Environment& env)
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
    border_colors->push_back(osg::Vec4d(1.0, 1.0, 1.0, 0.5));

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

    while(env.planes.size() > plane_group->getNumChildren()) {
        osg::ref_ptr<osg::Geode> geode = new osg::Geode;
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 5)); 

        geom->setColorBinding(osg::Geometry::BIND_OVERALL);
        geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        geode->addDrawable(geom.get());

        plane_group->addChild(geode);
    }

    for(unsigned i = 0; i < plane_group->getNumChildren(); i++) {
        updatePlaneNode(dynamic_cast<osg::Geode*>(plane_group->getChild(i)), env.planes[i]);
    }

    border_geom->setColorArray(border_colors.get());
    border_geom->setVertexArray(border_points.get());

    updated = false;
    property_updated = false;
}




void MapVisualization::updatePlaneNode(osg::Geode* geode, const uw_localization::Plane& plane)
{
    osg::Geometry* geom = dynamic_cast<osg::Geometry*>(geode->getDrawable(0));
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array; 
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

    osg::Vec3d pos(plane.position.x(), plane.position.y(), plane.position.z());
    osg::Vec3d sh(plane.span_horizontal.x(), plane.span_horizontal.y(), plane.span_horizontal.z());
    osg::Vec3d sv(plane.span_vertical.x(), plane.span_vertical.y(), plane.span_vertical.z());
    
    vertices->push_back(pos);
    vertices->push_back(pos + sh);
    vertices->push_back(pos + sh + sv);
    vertices->push_back(pos + sv);

    colors->push_back(osg::Vec4d(1.0, 1.0, 1.0, 0.3));

    geom->setVertexArray(vertices.get());
    geom->setColorArray(colors.get());
}


VizkitQtPlugin(MapVisualization)
}

