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
  Eigen::Vector2d position;
  Eigen::Vector2d span;
  double resolution;
  
  std::vector<E> grid;
  
public:
  
  
    /**
    * Constructor
    * @param position: middle of the grid
    * @param span: size of the grid
    * @param resolution: size of a single gridelement
    */
  GridMap(Eigen::Vector2d position, Eigen::Vector2d span, double resolution  ){
    
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
  virtual Eigen::Vector2d getGridCoord(double x, double y){
    
    Eigen::Vector2d result;
    
    //Check if coordinate is inside the grid
    if(x + position.x() > span.x() || x + position.x() < 0.0 
      || y + position.y() > span.y() || y + position.y() < 0.0)
        return Eigen::Vector2d(NAN, NAN);
    
    result.x() = (std::floor(x / resolution)) * resolution;
    result.y() = (std::floor(y / resolution)) * resolution;  
    
    return result;  
  }

  /**
   * Return the cell-ID of a given coordinate
   * Use this funktion for the grid-getter and setter methods
   * Returns -1, if coordinates are invalid
   */
  virtual Eigen::Vector2i getCellID(double x, double y){
    Eigen::Vector2i result;
    result.x() = ((x + position.x()) / span.x()) * (span.x()/resolution);
    result.y() = ((y + position.y()) / span.y()) * (span.y()/resolution);
    
    unsigned int entry = (result.y() * ((span.x()/resolution) +1 ) ) + result.x();
    if(result.x() < 0 || result.y() < 0 || entry >= grid.size() ){
      return Eigen::Vector2i(-1, -1);
    }
    
    return result;    
  }
  
  
  /**
   * Set an Gridelement
   * @param x,y: Grid IDs
   */
  //template<typename E>
  virtual void set(int x, int y, const E &val){
    grid[ (y * ( (span.x()/resolution) +1  ) ) + x ] = val;        
  }

  /**
   * Get a Gridelement
   * @param x,y: Grid Ids
   */
  //template<typename E>
  virtual E& get(int x, int y){
    int id = (y * ( (span.x()/resolution) +1 ) ) + x;

    return grid[ id];
  }

  /**
   * Get a list of gridcell, which intersects with a given scan
   * @param pos: Start position of the scan
   * @param angle: Angle of the scan, in world orientation and radion
   * @param min_dist: Start distanc of the scan. cells below this range will be ignored
   * @param max_dist: Maximal distance of the scan. 
   * @param infinite_scan: If true, there is no maximum distance restriction
   * @return: Vector of cell-coordinates
   */
  virtual std::vector<Eigen::Vector2d> getGridCells(Eigen::Vector2d pos, double angle, double min_dist, double max_dist, bool infinite_scan){
    
    std::vector<Eigen::Vector2d> result;
    Eigen::Vector2d lastCell;
    double cos_angle = std::cos(angle);
    double sin_angle = std::sin(angle);
    double map_size = span.norm();
        
    for(double dist = min_dist; dist < max_dist || ( infinite_scan && dist < map_size ); dist += resolution){
      
      Eigen::Vector2d v = getGridCoord(pos.x() + (cos_angle * dist), pos.y() + (sin_angle * dist) );
      
      if(v != lastCell && !isnan(v.x()) )
        result.push_back(v);
      
      lastCell = v;
      
    }   
    
    return result;
  }
  
};
  
}

#endif