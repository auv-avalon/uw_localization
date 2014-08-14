#ifndef UW_LOCALIZATION_DPSLAM_DPSLAMTYPES_HPP
#define UW_LOCALIZATION_DPSLAM_DPSLAMTYPES_HPP 
 
#include <base/samples/RigidBodyState.hpp>
#include <list>

namespace uw_localization{

 struct Feature{
   
   Feature():
    id(0), obstacle(false), obstacle_confidence(0.5), depth(NAN), obstacle_count(0), used(true) {}
   
   int64_t id;
   bool obstacle; //true, if ther eis an obstacle in the cell
   double obstacle_confidence; // confidence in the obstacle observation
   int obstacle_count;  //How many times the obstacle was observed
   double depth; //Depth of the cell
   double depth_variance; // Variance of the depth-observation
   bool used; //Is this feature still used
   
 };
  
 struct GridCell{
   
   GridCell():
    pos(base::Vector2d(NAN, NAN)), is_static(false) {}
      
   base::Vector2d pos;
   std::list<Feature> features;
   bool is_static; //Is there a static feature, e.g prior walls?
   
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
  std::list<std::pair<Eigen::Vector2d,int64_t > > depth_cells; //List of effected refs, with float-id of the assosiated ids
  std::list<std::pair<Eigen::Vector2d,int64_t > > obstacle_cells;
}; 

}

#endif