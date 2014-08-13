#ifndef UW_LOCALIZATION_DPSLAM_DPMAP_HPP
#define UW_LOCALIZATION_DPSLAM_DPMAP_HPP

#include <base/samples/RigidBodyState.hpp>
//#include "node_tree.hpp"
#include "../maps/grid_map.hpp"
#include "../maps/node_map.hpp"
#include "../types/environment.hpp"
#include "dp_types.hpp"
#include <vector>
#include <list>

namespace uw_localization{
  
 
 
 class DPMap : public GridMap<GridCell>{
   
 public:
   
   DPMap(base::Vector2d pos, base::Vector2d span, double resolution) : GridMap(pos, span, resolution), lastID(1) {}
   ~DPMap();
   
   virtual void initCoord(double x, double y){ setDepth(x, y, 0.0, 0.0, 0.0); }
   
   /**
    * Add static elements to the grid_map
    * Map-planes will be transformed to static grid cells;
    */
   void initalize_statics(NodeMap *map);
   
   /**
    * Return the depth in one cell
    * @param x,y: Coordinate of the cell
    * @param id: Unique id of the features
    * @return: Depth
    */
   double getDepth(double x, double y, int64_t id);
   
   /**
    * Set the depth on one coordinate
    * @param x,y: Coordinate of the cell
    * @param depth: Meassured depth
    * @param confidence: variance of the measurement
    * @param id: 
    * @return: Used feature id
    */
   int64_t setDepth(double x, double y, double depth, double variance, int64_t id);
   bool getObstacle(double x, double y, int64_t id);
   int64_t setObstacle(double x, double y, bool obstacle, double confidence, int64_t id);
   
   base::samples::Pointcloud getCloud(std::list<std::pair<Eigen::Vector2d,int64_t > > &depth_cells,
                                      std::list<std::pair<Eigen::Vector2d,int64_t > > &obstacle_cells,
                                      double confidence_threshold = 0.0);
   
   /**
    * Get a new, unique id
    */
   int64_t getNewID();
   
   /**
    * Return the feature, correspondig to the  id
    * If no w feature was foud, return empty feature with id 0
    * @param flag: set a flag at the feature
    */
   Feature getFeature(GridCell &cell, int64_t id, bool flag = false);
   
   void setFeature(GridCell &cell, int64_t id, Feature feature);
   
   /**
    * Remove all unused features
    */ 
   void reduceFeatures(double confidence_threshold);
   
   /**
    * Return observed obstacle cells
    * @param cells: list of cells, to be checked
    * @param id: list of all observations
    * @return: list of checked cells, which have obstacles and are part of the observation list
    */
   std::list< std::pair<Eigen::Vector2d, double > > getObservedCells(std::vector<Eigen::Vector2d> &cells, std::list<std::pair<Eigen::Vector2d,int64_t > > &ids);
   
 private:
   int64_t lastID;

 };
  
  
}

#endif