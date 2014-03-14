#include "MonitorVisualization.hpp"
#include "ParticleGeode.hpp"

namespace vizkit3d {

MonitorVisualization::~MonitorVisualization()
{
}


void MonitorVisualization::updateDataIntern(const uw_localization::Environment& env)
{
    data_env = env;
    map_changed = true;
}


void MonitorVisualization::updateDataIntern(const uw_localization::ParticleSet& p)
{
    while(particle_group->getNumChildren() < p.particles.size()) {
        osg::ref_ptr<vizkit3d::ParticleGeode> geode = new vizkit3d::ParticleGeode;
        particle_group->addChild(geode.get());
    }

   int i = 0;
    for(std::vector<uw_localization::Particle>::const_iterator it = p.particles.begin(); it != p.particles.end(); it++){
      
      if(it->main_confidence < p.particles[i].main_confidence)
	best_particle = i;
      
      i++;
    }    

    max_weight = p.particles[best_particle].main_confidence;

    for(unsigned i = 0; i < particle_group->getNumChildren(); i++) 
        dynamic_cast<vizkit3d::ParticleGeode*>(particle_group->getChild(i))->updateParticle(p.particles[i]);
}


void MonitorVisualization::updateDataIntern(const uw_localization::ParticleInfo& info) 
{
    for(unsigned i = 0; i < particle_group->getNumChildren(); i++) {
        vizkit3d::ParticleGeode* p = dynamic_cast<vizkit3d::ParticleGeode*>(particle_group->getChild(i));
        switch(info.type) {
            case 0:
                p->updateSonar(info.infos[i]);
                
                if(i == best_particle) {
                    p->showDesirePoint(property_desire_point);
                    p->showRealPoint(property_real_point);
                }

                break;
            default:
                break;
        }

    }
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
    
    root->addChild(border_geode.get());
    root->addChild(plane_group.get());
    root->addChild(particle_group.get());

    return root;
}


void MonitorVisualization::updateMainNode(osg::Node* node)
{
    if(!map_created || (property_dynamic_map && map_changed))
        renderEnvironment(data_env);

   if(map_created && particle_group->getNumChildren() > 0)
        renderParticles();
}

void MonitorVisualization::renderParticles()
{
    double max_z = data_env.right_bottom_corner.z();
    double min_z = data_env.left_top_corner.z();

    if(property_box)
        vizkit3d::ParticleGeode::setViz(vizkit3d::ParticleGeode::BOX, min_z, max_z, 0.1, max_weight);
    else
        vizkit3d::ParticleGeode::setViz(vizkit3d::ParticleGeode::LINE, min_z, max_z, 0.1, max_weight);

    for(unsigned i = 0; i < particle_group->getNumChildren(); i++) {
        dynamic_cast<vizkit3d::ParticleGeode*>(particle_group->getChild(i))->render();
    }
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

    if(property_gridlines) {
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
    }

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

    map_created = true;
    map_changed = false;
}




void MonitorVisualization::updatePlaneNode(osg::Geode* geode, const uw_localization::Plane& plane)
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


//VizkitQtPlugin(MonitorVisualization)
}

