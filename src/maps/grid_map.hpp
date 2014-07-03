#ifndef UW_LOCALIZATION_MAPS_GRIDMAP_HPP
#define UW_LOCALIZATION_MAPS_GRIDMAP_HPP

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>
#include <vector>

namespace uw_localization {
 
  struct GridElement{
    
    GridElement():
      depth_confidence(-1.0),
      depth(0.0),
      obstacle_confidence(-1.0),
      obstacle(false),
      pos(base::Vector2d(0,0)){}
    
    double depth_confidence;
    double depth;
    bool obstacle;
    double obstacle_confidence;
    base::Vector2d pos;
  };  
  
 class GridMap{
 public:
   
   /**
    * Constructor
    * @param position: middle of the grid
    * @param span: size of the grid
    * @param resolution: size of a single gridelement
    */
   GridMap(base::Vector2d position, base::Vector2d span, double resolution  );
   ~GridMap();
   
 double getDepth( double x, double y);
 void setDepth(double x, double y, double depth, double confidence);
 bool getObstacle(double x, double y);
 void setObstacle(double x, double y, bool obstalce, double confidence);
 base::samples::Pointcloud getCloud();
 
 
 private:
   std::vector<GridElement> grid;
   base::Vector2d position;
   base::Vector2d span;
   double resolution;
   
   void set(int x, int y, GridElement val);
   GridElement get(int x, int y);
   
 };
  
}


#endif