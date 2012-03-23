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

struct Particle {
    Particle()
    {}

    Particle(base::Position position, base::Time timestamp, double confidence) :
        position(position), timestamp(timestamp), confidence(confidence) 
    {}

    Particle(const Particle& p) :
        position(p.position), timestamp(p.timestamp), confidence(p.confidence)
    {}

    base::Position position;
    base::Time timestamp;

    double confidence;
};

}

#endif
