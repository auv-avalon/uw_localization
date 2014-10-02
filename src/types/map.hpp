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

namespace uw_localization {

  
  struct SimpleGridElement{
    
    SimpleGridElement() :
      depth(NAN), obstacle(false), obstacle_conf(0.0), static_object(false), flag(false), buoy_object(false), buoy_color(base::Vector3d::Zero()) {}

    double depth;
    bool obstacle;
    double obstacle_conf;
    bool static_object;
    bool flag;
    
    bool buoy_object;
    base::Vector3d buoy_color;
      
  };
  
  struct Feature{
    
    base::Vector3d color;
    base::Vector3d position;
    
  };
  
  struct SimpleGrid{
    
    
    base::Time time;
    std::vector<SimpleGridElement> grid;
    std::vector<Feature> feature_candidates;
    base::Vector2d origin;
    base::Vector2d span;
    double resolution;
    
    void init(base::Vector2d map_origin, base::Vector2d map_span, double map_resolution){
      origin = map_origin;
      span = map_span;
      resolution = map_resolution;
      
      grid.resize( ((span.x()/resolution) + 1) * ((span.y()/resolution) +1) );
    }
    
    bool getCell(double x, double y, SimpleGridElement &elem){
      
      int idx = ((x + origin.x()) / span.x()) * (span.x()/resolution);
      int idy = ((y + origin.y()) / span.y()) * (span.y()/resolution);
      
      unsigned int id = (idy * ( (span.x()/resolution) +1 ) ) + idx;
      
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
      
      unsigned int id = (idy * ( (span.x()/resolution) +1 ) ) + idx;
      
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
