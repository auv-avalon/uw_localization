/* ----------------------------------------------------------------------------
 * samples/map.h
 * written by Christoph Mueller, Mar 2012
 * University of Bremen
 * ----------------------------------------------------------------------------
*/

#ifndef UW_LOCALIZATION_SAMPLES_MAP_HPP_
#define UW_LOCALIZATION_SAMPLES_MAP_HPP_

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <string>
#include <limits>
#include <vector>
#include "../maps/depth_obstacle_grid.hpp"

namespace uw_localization {

  
  struct SimpleGridElement{
    
    SimpleGridElement() :
      depth(NAN), obstacle(false), obstacle_conf(0.0) {}

    double depth;
    bool obstacle;
    double obstacle_conf;  
      
      
  };
  
  struct SimpleGrid{
    
    
    base::Time time;
    std::vector<SimpleGridElement> grid;
    base::Vector2d origin;
    base::Vector2d span;
    double resolution;
    
    void init(base::Vector2d map_origin, base::Vector2d map_span, double map_resolution){
      origin = map_origin;
      span = map_span;
      resolution = map_resolution;
      
      grid.resize( (span.x()/resolution) * (span.y()/resolution));
    }
    
    bool getCell(double x, double y, SimpleGridElement &elem){
      
      int idx = ((x + origin.x()) / span.x()) * (span.x()/resolution);
      int idy = ((y + origin.y()) / span.y()) * (span.y()/resolution);
      
      int id = (idy * ( (span.x()/resolution) +1 ) ) + idx;
      
      if(id >= 0 && id < grid.size()){
        
        elem = grid[id];
        return true;        
      }
      else{
        return false;
      }      
      
    }
    
    bool setCell(double x, double y, SimpleGridElement &elem){
      
      int idx = ((x + origin.x()) / span.x()) * (span.x()/resolution);
      int idy = ((y + origin.y()) / span.y()) * (span.y()/resolution);
      
      int id = (idy * ( (span.x()/resolution) +1 ) ) + idx;
      
      if(id >= 0 && id < grid.size()){
        
        grid[id] = elem;
        return true;        
      }
      else{
        return false;
      }      
      
    }
    
    
  };


}

#endif 
