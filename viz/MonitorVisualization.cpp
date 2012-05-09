#include "MonitorVisualization.hpp"
#include "ParticleGeode.hpp"

namespace vizkit {

MonitorVisualization::~MonitorVisualization()
{
}


void MonitorVisualization::updateDataIntern(const uw_localization::Environment& env)
{
    data_env = env;
}


void MonitorVisualization::updateDataIntern(const uw_localization::ParticleSet& particles)
{
    data_particles = particles;
    particle_update = true;
}


osg::ref_ptr<osg::Node> MonitorVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> root = new osg::Group;
    plane_group = new osg::Group;
    particle_group = new osg::Group;

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

    root->addChild(border_geode);
    root->addChild(plane_group);
    root->addChild(particle_group);

    return root;
}


void MonitorVisualization::updateMainNode(osg::Node* node)
{
    if(property_dynamic_map || !map_created) 
        renderEnvironment(data_env);

    if(particle_update)
        renderParticles(data_particles);
}

void MonitorVisualization::renderParticles(const uw_localization::ParticleSet& set)
{
    double size = 0.1;
    double max_weight = set.particles[set.best_particle].main_confidence;
    double from = data_env.right_bottom_corner.z();
    double height = data_env.left_top_corner.z() - data_env.right_bottom_corner.z();

    if(particle_group->getNumChildren() == 0) {
        for(unsigned i = 0; i < set.particles.size(); i++) {
            osg::ref_ptr<ParticleGeode> geode = new ParticleGeode;
            particle_group->addChild(geode);

            if(property_box)
                geode->update_as_box(set.particles.at(i), size, max_weight);
            else 
                geode->update_as_line(set.particles.at(i), from, height, size, max_weight);
        }
    } else {
        for(unsigned i = 0; i < set.particles.size(); i++) { 
            ParticleGeode* p = dynamic_cast<ParticleGeode*>(particle_group->getChild(i));

            if(property_box)
                p->update_as_box(set.particles.at(i), size, max_weight);
            else 
                p->update_as_line(set.particles.at(i), from, height, size, max_weight);
        }
    }

    particle_update = false;
}


void MonitorVisualization::renderStatus()
{
}



void MonitorVisualization::renderEnvironment(const uw_localization::Environment& env)
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

    plane_group->removeChildren(0, plane_group->getNumChildren());
    for(unsigned i = 0; i < env.planes.size(); i++) {
        plane_group->addChild(createPlaneNode(env.planes[i]));
    }

    border_geom->setColorArray(border_colors.get());
    border_geom->setVertexArray(border_points.get());

    map_created = true;
}




osg::ref_ptr<osg::Geode> MonitorVisualization::createPlaneNode(const uw_localization::Plane& plane)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array; 
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;

    osg::Vec3d pos(plane.position.x(), plane.position.y(), plane.position.z());
    osg::Vec3d sh(plane.span_horizontal.x(), plane.span_horizontal.y(), plane.span_horizontal.z());
    osg::Vec3d sv(plane.span_vertical.x(), plane.span_vertical.y(), plane.span_vertical.z());
    
    vertices->push_back(pos);
    vertices->push_back(pos + sh);
    vertices->push_back(pos + sh + sv);
    vertices->push_back(pos + sv);

    while(colors->size() < vertices->size())
        colors->push_back(osg::Vec4d(1.0, 1.0, 1.0, 0.5));

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 5)); 
    geom->setVertexArray(vertices.get());
    geom->setColorArray(colors.get());

    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    geode->addDrawable(geom.get());

    return geode;
}


VizkitQtPlugin(MonitorVisualization)
}
