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
#include <vector>

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
