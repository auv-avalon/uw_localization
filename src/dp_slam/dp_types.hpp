#ifndef UW_LOCALIZATION_DPSLAM_DPSLAMTYPES_HPP
#define UW_LOCALIZATION_DPSLAM_DPSLAMTYPES_HPP 
 
#include <base/samples/RigidBodyState.hpp>
#include <list>
#include <limits>
#include <map>

namespace uw_localization{

  /**
   * Contains the confidence for an obstacle at a given depth-range
   */
 struct ObstacleDepthConfidence{
   
    double lower_border;
    double upper_border;
    double confidence;
    bool obstacle;
   
 };
  
  /**
   * One single obstacle feature
   */
 struct ObstacleFeature{
   
   ObstacleFeature():
    id(0), obstacle(false), obstacle_confidence(0.5), obstacle_count(0), used(true) {}
   
   int64_t id;
   bool obstacle; //true, if there is an obstacle in the cell
   double obstacle_confidence; // confidence in the obstacle observation
   std::vector<ObstacleDepthConfidence> obstacle_depth_confidence;
   int obstacle_count;  //How many times the obstacle was observed

   bool used; //Is this feature still used
   
   /**
    * Initialie the depth_confidence for this particles
    */
   void init_confidences(double min, double max, double resolution){
     
     for(double d = min; d < max; d += resolution){
       
       ObstacleDepthConfidence elem;
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
     
    for(std::vector<ObstacleDepthConfidence>::iterator it = obstacle_depth_confidence.begin(); it != obstacle_depth_confidence.end(); it++){
      
      if(it->obstacle && it->confidence > threshold){
        return true;
      }
      
    }
     
    return false;
          
   }
   
   
 };
 
  /**
   * One single depth feature
   * This could be an obstacle or a depth-observation
   */
 struct DepthFeature{
   
   DepthFeature():
    id(0), depth(NAN), depth_variance(std::numeric_limits<double>::infinity()), used(true) {}
   
   int64_t id;
   double depth; //Depth of the cell
   double depth_variance; // Variance of the depth-observation
   bool used; //Is this feature still used  
   
 };  
 
 /**
  * One grid cell
  * This could be a list of observed features or a static element
  */
 struct GridCell{
   
   GridCell():
    pos(base::Vector2d(NAN, NAN)), is_static(false), static_depth(NAN), static_depth_variance(INFINITY) {}
      
   base::Vector2d pos;
   std::map<int64_t, DepthFeature> depth_features;
   std::map<int64_t, ObstacleFeature> obstacle_features;
   bool is_static; //Is there a static feature, e.g prior walls?
   double static_depth; //Static depth_value, eg prior depth_map?
   double static_depth_variance;
   
 }; 
 
 struct SonarFeature{
   double dist; //Distance of the feature in m
   double confidence; //Confidence of the feature
 };
 
 struct SonarMeasurement{
   double angle; //Sonar yaw in global alllignment
   std::list<SonarFeature> features;
   
 };
  
/**
 * Explicit representation of position particles used by uw_particle_filter
 */
struct PoseSlamParticle {
  base::Position p_position;
  base::Vector3d p_velocity;
  base::Time timestamp;

  double main_confidence;
  bool valid;

  static base::samples::RigidBodyState* pose;
  
  /**
   * Map of all observations: key is a coordinate pair, value is a air of the observed cell and the observation-id
   */
  std::map< std::pair<double, double>, std::pair<Eigen::Vector2d,int64_t > > depth_cells; //List of effected refs, with float-id of the assosiated ids
  std::map< std::pair<double, double>, std::pair<Eigen::Vector2d,int64_t > > obstacle_cells;
}; 

}

#endif