#ifndef UW_LOCALIZATION_DPSLAM_DPMAP_HPP
#define UW_LOCALIZATION_DPSLAM_DPMAP_HPP

#include <base/samples/RigidBodyState.hpp>
//#include "node_tree.hpp"
#include "../maps/grid_map.hpp"
#include "../maps/node_map.hpp"
#include "../types/environment.hpp"
#include "../types/map.hpp"
#include "dp_types.hpp"
#include <vector>
#include <list>
#include <string>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>

namespace uw_localization{
  
 
 
 class DPMap : public GridMap<GridCell>{
   
 public:
   
   DPMap(base::Vector2d pos, base::Vector2d span, double resolution) : GridMap(pos, span, resolution), lastID(1) {}
   ~DPMap();
   
   virtual void initCoord(double x, double y){ setDepth(x, y, 0.0, 0.0, 0); }
   
   /**
    * Add static elements to the grid_map
    * Map-planes will be transformed to static grid cells;
    */
   void initalizeStatics(NodeMap *map);
   
   /**
    * Add static depths to the grid map
    * Filename should refere to a yaml filename
    */
   bool initalizeStaticDepth(const std::string &filename); 
   
   /**
    * Set resolution and span for vertical observation confidences
    */
   void initDepthObstacleConfig(double min_depth, double max_depth, double depth_resolution);
   
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
   
   void setStaticDepth(double x, double y, double depth, double variance);
   
   bool getObstacle(double x, double y, int64_t id);
   
   /**
    * Set an obstacle
    * @param x,y : coordinate of the obstacle
    * @param obstacle: true, if we saw an obstacle on this position
    * @param confidence: confidence of this observation
    * @param min_depth: minimum depth, at which this observation holds
    * @param max_depth: maximum_depth, at which this observation holds
    * @param id: id, of the last observation for this cell
    */
   int64_t setObstacle(double x, double y, bool obstacle, double confidence, double min_depth, double max_depth, int64_t id);
   
   /**
    * Touch a feature at a given cell, to propose, that the feature is used
    */
   void touchDepthFeature(double x, double y, int64_t id);
   void touchObstacleFeature(double x, double y, int64_t id);
   
   /**
    * Get a pointcloud-representation of the map
    * @param depth_cell: list of cells and observation-ids, coreponding to one particle
    * @param obstacle_cells: list of cell and observation-id, coresponding to one particle
    * @param confidence_threshold: observations, with confidence below this threshold will be ignored
    * @param count_threshold: ignore observation_confidence, if a obstacle was observed this often
    */
   base::samples::Pointcloud getCloud(std::map< std::pair<double, double>, std::pair<Eigen::Vector2d,int64_t > > &depth_cells,
                                      std::map< std::pair<double, double>, std::pair<Eigen::Vector2d,int64_t > > &obstacle_cells,
                                      double confidence_threshold = 0.0, int count_threshold = 0);
   
   /**
    * Params like getCloud
    * Return is the maximum number of features per cell
    */
   int getSimpleGrid( uw_localization::SimpleGrid &grid,
                                      std::map< std::pair<double, double>, std::pair<Eigen::Vector2d,int64_t > > &depth_cells,
                                      std::map< std::pair<double, double>, std::pair<Eigen::Vector2d,int64_t > > &obstacle_cells,
                                      double confidence_threshold = 0.0, int count_threshold = 0);   
   
   
   /**
    * Get a new, unique id
    */
   int64_t getNewID();
   
   /**
    * Return the feature, correspondig to the  id
    * If no w feature was foud, return empty feature with id 0
    * @param flag: set a flag at the feature
    */
   DepthFeature getDepthFeature(GridCell &cell, int64_t id, bool flag = false);
   ObstacleFeature getObstacleFeature(GridCell &cell, int64_t id, bool flag = false);
   
   void setDepthFeature(GridCell &cell, int64_t id, DepthFeature &feature);
   void setObstacleFeature(GridCell &cell, int64_t id, ObstacleFeature &feature);
   
   /**
    * Remove all unused features
    * @param confidence_threshold: features below this threshold will be removed
    * @param count_threshold: features with a count above this threshold will be saved
    */ 
   void reduceFeatures(double confidence_threshold, int count_threshold);
   
   /**
    * Return observed obstacle cells
    * @param cells: list of cells, to be checked
    * @param id: list of all observations
    * @return: list of checked cells, which have obstacles and are part of the observation list
    */
   std::list< std::pair<Eigen::Vector2d, double > > getObservedCells(std::vector<Eigen::Vector2d> &cells,
                                                                     std::map< std::pair<double,double>, std::pair<Eigen::Vector2d,int64_t > > &ids);
   
   
   /**
    * Sets the confidence of an obstacle, based on an observation and observation depth
    * @param depth_vector: vector of depth confidences
    * @param min: minimum depth of the observation
    * @param max: maximum depth of the observation
    * @param confidence: confidence of the observation
    * @param obstacle: did we saw an obstacle in this cell
    */
   void setObstacleDepthConfidence(std::vector<ObstacleDepthConfidence> &depth_vector, double min, double max, double confidence, bool obstacle);
   
   
 private:
   int64_t lastID;
   
   double max_depth;
   double min_depth;
   double depth_resolution;

 };
  
  
}

#endif