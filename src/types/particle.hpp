/* ----------------------------------------------------------------------------
 * samples/particles.h
 * written by Christoph Mueller, Oct 2011
 * University of Bremen
 * ----------------------------------------------------------------------------
*/

#ifndef UW_LOCALIZATION_SAMPLES_PARTICLES_H_
#define UW_LOCALIZATION_SAMPLES_PARTICLES_H_

#include <base/pose.h>
#include <base/time.h>
#include <base/samples/RigidBodyState.hpp>
#include <vector>
//#include "../dp_slam/node_tree.hpp"
//#include "../dp_slam/dp_slam.hpp"

namespace uw_localization {

/**
 * General representation for particles used by all filters and visualizations
 */
struct Particle {
    /** current estimated position */
    base::Position position;

    /** current estimated velocity */
    base::Vector3d velocity;

    /** current heading / yaw for this particle */
    double yaw;
 
    /** current believe for this particle */
    double main_confidence;
};


/**
 * Explicit representation of position particles used by uw_particle_filter
 *
struct PoseSlamParticle {
  base::Position p_position;
  base::Vector3d p_velocity;
  base::Time timestamp;

  double main_confidence;

  static base::samples::RigidBodyState* pose;
  std::vector<GridCell*> grid_refs; 
};*/


/**
 * Explicit representatiom of of position particles with dp_slam
 */

struct PoseParticle {
  base::Position p_position;
  base::Vector3d p_velocity;
  base::Time timestamp;

  double main_confidence;

  static base::samples::RigidBodyState* pose;
  
};


/**
 * General representation for a particle set
 */
struct ParticleSet {
    /** current timestamp for this particle set */
    base::Time timestamp;

    /** current particle set for this state */ 
    std::vector<Particle> particles;
};


}
#endif
