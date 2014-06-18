#ifndef UW_LOCALIZATION_MAPS_GRIDMAP_HPP
#define UW_LOCALIZATION_MAPS_GRIDMAP_HPP

#include <base/samples/RigidBodyState.hpp>
#include <vector>

namespace uw_localization {
 
  struct GridElement{
    
    GridElement():
      confidence(0.0),
      depth(0.0),
      pos(base::Vector2d(0,0)){}
    
    double confidence;
    double depth;
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
 std::vector<base::Vector3d> getCloud();
 
 
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