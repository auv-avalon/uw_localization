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
  virtual Eigen::Vector2i getCellID(double x, double y){
    Eigen::Vector2i result;
    result.x() = ((x + position.x()) / span.x()) * (span.x()/resolution);
    result.y() = ((y + position.y()) / span.y()) * (span.y()/resolution);
    
    int entry = (result.y() * (span.x()/resolution) ) + result.x();
    if(entry >= grid.size() || entry < 0){
      return Eigen::Vector2i(NAN, NAN);
    }
    
    return result;    
  }
  
  
  /**
   * Set an Gridelement
   * @param x,y: Grid IDs
   */
  //template<typename E>
  virtual void set(int x, int y, const E &val){
    grid[ (y * (span.x()/resolution) ) + x ] = val;        
  }

  /**
   * Get a Gridelement
   * @param x,y: Grid Ids
   */
  //template<typename E>
  virtual E& get(int x, int y){
    int id = (y * (span.x()/resolution) ) + x;

    return grid[ id];
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
    
    for(double dist = min_dist; dist < max_dist; dist += resolution){
      
      base::Vector2d v = getGridCoord(pos.x() + (cos_angle * dist), pos.y() + (sin_angle * dist) );
      
      if(v != lastCell && !isnan(v.x()) )
        result.push_back(v);
      
      lastCell = v;
      
    }   
    
    return result;
  }
  
};
  
}

#endif