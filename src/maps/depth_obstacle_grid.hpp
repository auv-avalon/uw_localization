#ifndef UW_LOCALIZATION_MAPS_DEPTHOBSTACLEGRID_HPP
#define UW_LOCALIZATION_MAPS_DEPTHOBSTACLEGRID_HPP

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <utility>
#include "grid_map.hpp"

namespace uw_localization {
 
  struct GridElement{
    
    GridElement():
      depth_confidence(-1.0),
      depth(0.0),
      obstacle_confidence(-1.0),
      obstacle(false),
      obstacle_weight(1.0),
      pos(base::Vector2d(0,0)){}
    
    double depth_confidence;
    double depth;
    bool obstacle;
    double obstacle_confidence;
    double obstacle_weight;
    base::Vector2d pos;
  };  
  
 /**
  * Spezialization of the grid map, saving depth and obstacle values in each cells
  */
 class DepthObstacleGrid : public GridMap<GridElement>{
 public:   

 DepthObstacleGrid() : GridMap<GridElement>( base::Vector2d(0.0, 0.0), base::Vector2d(1.0, 1.0), 1.0) {} 
   
   
    /**
    * Constructor
    * @param position: middle of the grid
    * @param span: size of the grid
    * @param resolution: size of a single gridelement
    */
  DepthObstacleGrid(base::Vector2d position, base::Vector2d span, double resolution  ) : GridMap<GridElement>(position, span, resolution) {}
  
  ~DepthObstacleGrid();
  
 virtual void initCoord(double x, double y){ setDepth(x, y, 0.0, 0.0); }
  
 double getDepth( double x, double y);
 void setDepth(double x, double y, double depth, double confidence);
 bool getObstacle(double x, double y);
 void setObstacle(double x, double y, bool obstalce, double confidence);
 base::samples::Pointcloud getCloud();
 
 /**
  * Reduce the weight of all cells by a constant value
  */
 void reduce_weights(double val); 

   
 };
  
}


#endif