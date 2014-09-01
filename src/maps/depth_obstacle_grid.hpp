#ifndef UW_LOCALIZATION_MAPS_DEPTHOBSTACLEGRID_HPP
#define UW_LOCALIZATION_MAPS_DEPTHOBSTACLEGRID_HPP

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <utility>
#include "grid_map.hpp"
#include "../types/map.hpp"

namespace uw_localization {
 
 struct ObstacleConfidence{
   
    double lower_border;
    double upper_border;
    double confidence;
    bool obstacle;
   
 };  
  
  
  struct GridElement{
    
    GridElement():
      depth_variance(INFINITY),
      depth(NAN),
      obstacle_confidence(-1.0),
      obstacle(false),
      obstacle_weight(1.0),
      obstacle_count(0),
      pos(base::Vector2d(0,0)){}
    
    double depth_variance;
    double depth;
    bool obstacle;
    double obstacle_confidence;
    std::vector<ObstacleConfidence> obstacle_depth_confidence;
    double obstacle_weight;
    int obstacle_count;
    base::Vector2d pos;
    
   /**
    * Initialie the depth_confidence for this particles
    */
   void init_confidences(double min, double max, double resolution){
     
     obstacle_depth_confidence.clear();
     
     for(double d = min; d < max; d += resolution){
       
       ObstacleConfidence elem;
       elem.confidence = 0.5;
       elem.lower_border = d;
       elem.upper_border = d + resolution;
       elem.obstacle = false;
       
       obstacle_depth_confidence.push_back(elem);
       
     }
     
   }
   
   /**
    * returns true, if one depth_cell is above the confidence-threshold
    */ 
   bool is_obstacle(double threshold){
     
    for(std::vector<ObstacleConfidence>::iterator it = obstacle_depth_confidence.begin(); it != obstacle_depth_confidence.end(); it++){
      
      if(it->obstacle && it->confidence > threshold){
        return true;
      }
      
    }
     
    return false;
          
   }    
    
    
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
 
 void initDepthObstacleConfig(double min_depth, double max_depth, double depth_resolution);
 
 void setThresholds(double confidence_threshold, double count_threshold);
  
 double getDepth( double x, double y);
 void setDepth(double x, double y, double depth, double variance);
 bool getObstacle(double x, double y);
 void setObstacle(double x, double y, bool obstalce, double confidence);
 base::samples::Pointcloud getCloud();
 void getSimpleGrid(uw_localization::SimpleGrid &simple_grid, double confidence_threshold = 0.0, int count_threshold = 0);
 
 void setObstacleDepthConfidence(std::vector<ObstacleConfidence> &depth_vector, double min, double max, double confidence, bool obstacle); 
 
 /**
  * Reduce the weight of all cells by a constant value
  */
 void reduce_weights(double val); 
 
 
 private:
   double max_depth;
   double min_depth;
   double depth_resolution;

   double obstacle_confidence_threshold;
   double obstacle_count_threshold;
  
   
   
   
 };
  
}


#endif