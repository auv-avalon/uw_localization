#ifndef UW_LOCALIZATION_SONARPOINTVISUALIZATION_HPP
#define UW_LOCALIZATION_SONARPOINTVISUALIZATION_HPP

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/Geometry>
#include <base/samples/rigid_body_state.h>
#include <uw_localization/types/info.hpp>

namespace vizkit {

class SonarPointVisualization
    : public vizkit::Vizkit3DPlugin<uw_localization::PointInfo>,
      boost::noncopyable
{
   Q_OBJECT
   Q_PROPERTY(double min_z READ getMinZ WRITE setMinZ)
   Q_PROPERTY(double max_z READ getMaxZ WRITE setMaxZ)
   Q_PROPERTY(int history READ getHistory WRITE setHistory)

public:
   SonarPointVisualization() {
       VizPluginRubyAdapter(SonarPointVisualization, uw_localization::PointInfo, PointInfo);

       property_min_z = -1.0;
       property_max_z = 0.0;
       property_history = 30;
       updated = false;
   }

   double getMinZ() const { return property_min_z; }
   double getMaxZ() const { return property_max_z; }
   void   setMinZ(double p) { property_min_z = p; }
   void   setMaxZ(double p) { property_max_z = p; }
   void   setHistory(int p) { property_history = p; }
   int    getHistory() const { return property_history; }

protected:
   virtual osg::ref_ptr<osg::Node> createMainNode();
   virtual void updateMainNode(osg::Node* node);
   virtual void updateDataIntern(const uw_localization::PointInfo& point);

private:
   double property_min_z;
   double property_max_z;
   int property_history;
   bool updated;

   uw_localization::PointInfo point;
   std::list<uw_localization::PointInfo> history;

   osg::ref_ptr<osg::Geometry> sonar_geom;
   osg::ref_ptr<osg::Vec4Array> sonar_color;
   osg::ref_ptr<osg::Vec3Array> sonar_vertices;
   osg::ref_ptr<osg::DrawArrays> sonar_draw;

   osg::ref_ptr<osg::Geometry> desire_geom;
   osg::ref_ptr<osg::Vec4Array> desire_color;
   osg::ref_ptr<osg::Vec3Array> desire_vertices;
   osg::ref_ptr<osg::DrawArrays> desire_draw;

   osg::ref_ptr<osg::Geometry> hist_geom;
   osg::ref_ptr<osg::Vec4Array> hist_color;
   osg::ref_ptr<osg::Vec3Array> hist_vertices;
   osg::ref_ptr<osg::DrawArrays> hist_draw;
};


}

#endif // UW_LOCALIZATION_SONARPOINTVISUALIZATION_HPP



