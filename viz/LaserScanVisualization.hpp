#ifndef LASERSCAN_VISUALIZATION_H_
#define LASERSCAN_VISUALIZATION_H_

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <base/samples/pointcloud.h>
#include <base/samples/laser_scan.h>
#include <base/samples/rigid_body_state.h>

namespace vizkit3d {

class LaserScanVisualization
    : public vizkit3d::Vizkit3DPlugin<base::samples::LaserScan>,
      public vizkit3d::VizPluginAddType<base::samples::RigidBodyState>,
      boost::noncopyable
{
    Q_OBJECT
    Q_PROPERTY(double height READ getScanHeight WRITE setScanHeight)
    Q_PROPERTY(double yaw_offset READ getYawOffset WRITE setYawOffset)
    Q_PROPERTY(int buffersize READ getBufferSize WRITE setBufferSize)
    Q_PROPERTY(bool beam READ isBeam WRITE setBeam)


    public:
        LaserScanVisualization();
        ~LaserScanVisualization();

        double getScanHeight() const { return scan_height; }
        void   setScanHeight(double height) { scan_height = height; }

        void   setColor(base::Vector3d const& color) { this->color = color; }

        int    getBufferSize() const { return scan_buffer_size; }
        void   setBufferSize(int size) { scan_buffer_size = size; }

        double getYawOffset() const { return yawoffset; }
        void   setYawOffset(double yaw) { yawoffset = yaw; }

        bool   isBeam() const { return show_beam; }
        void   setBeam(bool enable) { show_beam = enable; }
    
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(const base::samples::LaserScan& scan);
        virtual void updateDataIntern(const base::samples::RigidBodyState& pose);

    private:
        bool new_laserscan_received;
        std::list<base::Vector3d> buffer;
        base::samples::Pointcloud point_cloud;
        base::samples::RigidBodyState pose;
        size_t scan_buffer_size;
        bool show_beam;

        double yawoffset;
        
        unsigned int latest_sample_index;
        
        double scan_height;
        base::Vector3d color;

        osg::ref_ptr<osg::Geometry> scan_geom;
        osg::ref_ptr<osg::Geometry> beam_geom;
        osg::ref_ptr<osg::DrawArrays> beam_draw_arrays;
        osg::ref_ptr<osg::DrawArrays> scan_draw_arrays;
        osg::ref_ptr<osg::Vec3Array> beam_points;
        osg::ref_ptr<osg::Vec3Array> scan_points;
        osg::ref_ptr<osg::Vec4Array> scan_color_array;
        osg::ref_ptr<osg::Vec4Array> beam_color_array;
};


}

#endif
