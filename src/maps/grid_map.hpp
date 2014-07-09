#ifndef UW_LOCALIZATION_MAPS_GRIDMAP_HPP
#define UW_LOCALIZATION_MAPS_GRIDMAP_HPP

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Pointcloud.hpp>
#include <vector>
#include <cmath>
#include <iostream>
#include <utility>

namespace uw_localization{

/**
 * Abstract grid map class
 * Class already provides cell-getter and setter, discretisation methods and a scan simulation
 */  
template<typename E>
class GridMap{
  
protected:
  base::Vector2d position;
  base::Vector2d span;
  double resolution;
  
  std::vector<E> grid;
  
public:
  
  
    /**
    * Constructor
    * @param position: middle of the grid
    * @param span: size of the grid
    * @param resolution: size of a single gridelement
    */
  GridMap(base::Vector2d position, base::Vector2d span, double resolution  ){
    
    std::cout << "Set up grid map:" << std::endl;
    std::cout << "Center: " << position.transpose() << std::endl;
    std::cout << "Span: " << span.transpose() << std::endl;
    std::cout << "Resolution: " << resolution << std::endl;
    
    this->position = position;
    this->span = span;
    this->resolution = resolution;
    
    grid.resize( (span.x()/resolution) * (span.y()/resolution));

    std::cout << "Map size: " << grid.size() << std::endl;
    
  }


  ~GridMap(){
      
  }
  
  /**
   * Init a grid cell at a given position
   */
  virtual void initCoord(double x, double y) = 0;  
  
  /**
   * Init the entire grid
   */
  virtual void initGrid(){
    
   for( double x = 0.0; x < span.x(); x = x + resolution){
     
     for( double y = 0.0; y < span.y(); y = y + resolution){
       
       initCoord(x - position.x(), y - position.y());
       
     }
   }
    
    
  }

  
  /**
   * Return an discrete grid-coordinate
   */
  virtual base::Vector2d getGridCoord(double x, double y){
    
    base::Vector2d result;
    
    //Check if coordinate is inside the grid
    if(x + position.x() > span.x() || x + position.x() < 0.0 
      || y + position.y() > span.y() || y + position.y() < 0.0)
        return base::Vector2d(NAN, NAN);
    
    result.x() = (std::floor(x / resolution)) * resolution;
    result.y() = (std::floor(y / resolution)) * resolution;  
    
    return result;  
  }

  /**
   * Return the cell-ID of a given coordinate
   * Use this funktion for the grid-getter and setter methods
   */
  virtual std::pair<int, int> getCellID(double x, double y){
    std::pair<int, int> result;
    result.first = ((x + position.x()) / span.x()) * (span.x()/resolution);
    result.second = ((y + position.y()) / span.y()) * (span.y()/resolution);
    
    return result;    
  }
  
  
  /**
   * Set an Gridelement
   * @param x,y: Grid IDs
   */
  //template<typename E>
  virtual void set(int x, int y, E val){
    int id = (y * (span.x()/resolution) ) + x;
    if( id < grid.size() && id >= 0)
      grid[ (y * (span.x()/resolution) ) + x ] = val;
    else{
      //std::cout << "Invalid get:   x: " << x << " y: " << y << " id: " << id << " girdsize: " << grid.size() << std::endl;
    }
    
  }

  /**
   * Get a Gridelement
   * @param x,y: Grid Ids
   */
  //template<typename E>
  virtual E get(int x, int y){
    int id = (y * (span.x()/resolution) ) + x;
    
    if( id  < grid.size() && id >= 0)
      return grid[ (y * (span.x()/resolution) ) + x];
    else{
      //std::cout << "Invalid set:  x: " << x << " y: " << y << " id: " << id << " gridsize: " << grid.size() << std::endl;
      return E();
    }
  }

  /**
   * Get a list of gridcell, which intersects with a given scan
   * @param pos: Start position of the scan
   * @param angle: Angle of the scan, in world orientation and radion
   * @param min_dist: Start distanc of the scan. cells below this range will be ignored
   * @param max_dist: Maximal distance of the scan. 
   * @return: Vector of cell-coordinates
   */
  virtual std::vector<base::Vector2d> getGridCells(Eigen::Vector2d pos, double angle, double min_dist, double max_dist){
    
    std::vector<base::Vector2d> result;
    base::Vector2d lastCell;
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    std::cout << "Get grid cells: pos " << pos.transpose() << " - angle " << angle << " - min_dist " << min_dist << " -max_dist " << max_dist << std::endl;
    for(double dist = min_dist; dist < max_dist; dist += resolution){
      
      base::Vector2d v = getGridCoord(pos.x() + (cos_angle * dist), pos.y() + (sin_angle * dist) );
      std::cout << "V: " << v.transpose() << std::endl;
      if(v != lastCell && !isnan(v.x()) )
        result.push_back(v);
      
      lastCell = v;
      
    }   
    
    return result;
  }
  
};
  
}

#endif