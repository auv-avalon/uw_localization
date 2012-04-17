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

   /** current heading / yaw for this particle */
    double yaw;

    /** sub_confidences (perception, random, max_distance) **/
    base::Vector3d part_confidences;
 
    /** current believe for this particle */
    double main_confidence;
};


/**
 * General representation for a particle set
 */
struct ParticleSet {
    /** current timestamp for this particle set */
    base::Time timestamp;

    /** weights for forming main_confidences */
    base::Vector3d weights;

    /** effective sample size for controlling resampling */
    double effective_sample_size;
    
    /** current particle set for this state */ 
    std::vector<Particle> particles;

    /** index of particle with maximum main confidence of the particle set */
    unsigned int max_particle_index;
};


}
#endif
