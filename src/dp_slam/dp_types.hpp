#ifndef UW_LOCALIZATION_DPSLAM_DPSLAMTYPES_HPP
#define UW_LOCALIZATION_DPSLAM_DPSLAMTYPES_HPP 
 
#include <base/samples/RigidBodyState.hpp>
#include <list>

namespace uw_localization{

 struct Feature{
   
   Feature():
    id(0) {}
   
   int64_t id;
   bool obstacle;
   double obstacle_confidence;
   double depth;
   double depth_confidence;
   bool used;
   
 };
  
 struct GridCell{
   
   GridCell():
    pos(base::Vector2d(NAN, NAN)) {}
   
   base::Vector2d pos;
   std::list<Feature> features;   
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