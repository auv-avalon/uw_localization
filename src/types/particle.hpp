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

namespace uw_localization {

/**
 * General representation for particles used by all filters and visualizations
 */
struct Particle {
    Particle() : position(0.0, 0.0, 0.0), yaw(0.0), norm_weight(0.0)
    {}

    Particle(const Particle& p) :
        position(p.position), yaw(p.yaw), norm_weight(p.norm_weight)
    {}

    /** current estimated position */
    base::Position position;

    /** current heading / yaw for this particle */
    double yaw;

    /** current believe for this particle */
    double norm_weight;
};


/**
 * General representation for a particle set
 */
struct ParticleSet { 
    /** current timestamp for this particle set */
    base::Time timestamp;

    /** current particle set for this state */ 
    std::vector<Particle> particles;

    /** index of particle with maximum weight of the particle set */
    unsigned int max_particle_index;

    /** probability for a plausible state estimation (relating kidnapping problem) */
    double confidence;
};

}
#endif
