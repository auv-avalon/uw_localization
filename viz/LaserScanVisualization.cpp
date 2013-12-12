#include "LaserScanVisualization.hpp"
#include <algorithm>

namespace vizkit3d {

LaserScanVisualization::LaserScanVisualization() 
    : new_laserscan_received(false), scan_buffer_size(128), show_beam(false), 
      yawoffset(0.0), latest_sample_index(0), scan_height(2.0), color(1.0, 0.62, 0.0)
{
    VizPluginRubyAdapter(LaserScanVisualization, base::samples::LaserScan, LaserScan);
    VizPluginRubyAdapter(LaserScanVisualization, base::samples::RigidBodyState, Pose);
    VizPluginRubyMethod(LaserScanVisualization, base::Vector3d, setColor);
    pose.invalidate();
}


LaserScanVisualization::~LaserScanVisualization()
{}


osg::ref_ptr<osg::Node> LaserScanVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> root = new osg::Group;
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    scan_color_array = new osg::Vec4Array;
    beam_color_array = new osg::Vec4Array;

    scan_color_array->push_back(osg::Vec4(color.x(), color.y(), color.z(), 1.0f));
    beam_color_array->push_back(osg::Vec4(color.x(), color.y(), color.z(), 1.0f));
    
    // setup point cloud
    scan_geom = new osg::Geometry;
    scan_points = new osg::Vec3Array;
    scan_draw_arrays = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, scan_points->size()); 

    scan_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    scan_geom->setVertexArray(scan_points);
    scan_geom->setColorArray(scan_color_array);
    scan_geom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    scan_geom->addPrimitiveSet(scan_draw_arrays.get());

    // setup beam lines for laserscans
    beam_geom = new osg::Geometry;
    beam_points = new osg::Vec3Array;
    beam_draw_arrays = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, beam_points->size());

    beam_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    beam_geom->setVertexArray(beam_points);
    beam_geom->setColorArray(beam_color_array);
    beam_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    beam_geom->addPrimitiveSet(beam_draw_arrays.get());

    geode->addDrawable(scan_geom.get());
    geode->addDrawable(beam_geom.get());

    root->addChild(geode);

    return root;
}


void LaserScanVisualization::updateMainNode(osg::Node* node)
{
    if(new_laserscan_received) {
        new_laserscan_received = false;

        scan_points->clear();
        scan_color_array->clear();
  
        beam_points->clear();
        beam_draw_arrays->setCount(beam_points->size());

        if(show_beam)
            beam_color_array->clear();
        
        std::list<base::Vector3d>::const_iterator pos;
        unsigned int index = 0;
        for(pos = buffer.begin(); pos != buffer.end(); pos++, index++) {
            osg::Vec3d vec(pos->x(), pos->y(), pos->z());
            scan_points->push_back(vec);
            scan_points->push_back(vec + osg::Vec3d(0.0, 0.0, scan_height));

            scan_color_array->push_back(osg::Vec4(color.x(), color.y(), color.z(), 1.0f));

            if(show_beam && latest_sample_index <= index) {
                osg::Vec3d to_pos(pos->x(), pos->y(), pos->z());

                osg::Vec3d from_pos(pose.position.x(), pose.position.y(), 0.0);

                beam_points->push_back(to_pos);
                beam_points->push_back(from_pos);
                beam_color_array->push_back(osg::Vec4(color.x(), color.y(), color.z(), 1.0f));
            }
        }

        scan_draw_arrays->setCount(scan_points->size());
        scan_geom->setVertexArray(scan_points);
        scan_geom->setColorArray(scan_color_array);

        beam_geom->setColorArray(beam_color_array);

        if(beam_points->size() > 0) {
            beam_draw_arrays->setCount(beam_points->size());
            beam_geom->setVertexArray(beam_points);
        }

        latest_sample_index = buffer.size();
    }
}


void LaserScanVisualization::updateDataIntern(const base::samples::LaserScan& scan)
{
    std::vector<Eigen::Vector3d> points;
    scan.convertScanToPointCloud(points, pose);

    Eigen::AngleAxis<double> offset(yawoffset, Eigen::Vector3d::UnitZ());

    if(points.size() > 0) {
        new_laserscan_received = true;
    } else
        return;

    for(unsigned int i = 0; i < points.size(); i++)
        buffer.push_back(offset * points[i]);

    while(buffer.size() > scan_buffer_size) {
        buffer.pop_front();
        latest_sample_index = buffer.size() - 1;
    }
}


void LaserScanVisualization::updateDataIntern(const base::samples::RigidBodyState& pose)
{
    this->pose = pose;
}


VizkitQtPlugin(LaserScanVisualization)
} // namespace vizkit
