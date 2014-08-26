#ifndef UWLOCALIZATION_SimpleGridVisualization_H
#define UWLOCALIZATION_SimpleGridVisualization_H

#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <uw_localization/types/particle.hpp>
#include <uw_localization/types/map.hpp>

#include <osg/Node>
#include <osg/Geometry>

namespace vizkit3d
{

/**
 * Vizkit plugin to visualize particle sets
 * 
 */
class SimpleGridVisualization : public vizkit3d::Vizkit3DPlugin< uw_localization::SimpleGrid >, boost::noncopyable
{    
    Q_OBJECT
    Q_PROPERTY(QColor defaultFeatureColor READ getDefaultFeatureColor WRITE setDefaultFeatureColor)
    
    public:
        SimpleGridVisualization();
        
        Q_INVOKABLE void updateSimpleGrid( const uw_localization::SimpleGrid& sample )
        {
	  return updateData(sample); }
                
    public slots:
        QColor getDefaultFeatureColor();
        void setDefaultFeatureColor(QColor color);
        
    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        void updateDataIntern ( const uw_localization::SimpleGrid& data );
        void updateDataIntern(const std::vector< base::Vector3d >& data);
                
    private:
	osg::Vec4f hsv2rgb(double h, double s, double v);
        uw_localization::SimpleGrid grid;
        osg::Vec4f default_feature_color;
        std::vector< base::Vector3d > channelInfos;
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> pointGeom;
        osg::ref_ptr<osg::Vec4Array> color;
        bool updated;
};

}
#endif
