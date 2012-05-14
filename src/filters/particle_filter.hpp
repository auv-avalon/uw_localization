/* ----------------------------------------------------------------------------
 * particle_filter.h
 * written by Christoph Mueller, Oct 2011
 * University of Bremen
 * ----------------------------------------------------------------------------
*/
#ifndef UW_LOCALIZATION__PARTICLEFILTER_H_
#define UW_LOCALIZATION__PARTICLEFILTER_H_

#include <vector>
#include <assert.h>
#include <machine_learning/RandomNumbers.hpp>
#include <base/samples/rigid_body_state.h>
#include <uw_localization/types/particle.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace uw_localization {

/**
 * Generic interface for sending internal information to e.g. an orocos task
 * Best practise for an orocos task is implementing this interface.
 * In that way all particle filter are completely independent from the orocos layer.
 */
template<typename D>
class DebugWriter {
 public:
    virtual void write(const D& debug_values) = 0;
};


template<typename P, typename Z, typename M>
class Perception { 
 public:
    /**
     * represents the measurement model the particle filter use for weighting each state particle
     * and for priorizing comming resampling processes
     * the confidence of each particle is given by the likelihood p(zt | xt, m)
     *
     * \param sensor current representation for a measurement of a sensor
     * \param updated state current representation of a state hypothesis (from a single particle)
     * \param map current representation for a given map hypothesis
     * \return updated state and probability of a plausible particle set (can be a constant if not used)
     */
    virtual double perception(const P& p, const Z& sensor, const M& map) {
	throw new std::runtime_error("perception model is required");
	return 0.0;
    }
};




template<typename P, typename U>
class Dynamic {
 public:
    /**
     * represents the motion model the particle filter use for drawing a sample of the 
     * probability xt[m] ~ P(xt | xt-1, ut). Keep in mind xt[m] is just a sample
     * and must be involved with a gaussian bias (requirement for particle filters)
     *
     * \param perception type (useful for differ perceptions with same update type)
     * \param state updates current representation of a state hypothesis (from a single particle)
     * \param motion current representation of a motion call
     * \returns updated state sample depending on we were in state xt-1 and we move with ut and
     *     measurement plausibility (can be a constant if not used)
     */
    virtual void dynamic(P& state, const U& motion) = 0;

    /**
     * get current timestamp for this motion command
     */
    virtual const base::Time& getTimestamp(const U& motion) = 0;
};


/**
 * Particle Helper class for concret particle filters
 */
class ParticleBase {
 public:
   /**
     * get a general position representation from a given abstract pose particles
     *
     * \param state of type P
     * \return a simple vector with three dimension
     */
    virtual base::Position position() const = 0;

    /**
     * get a general orientation representation from a given abstract poste particle
     * \param state of type P
     * \return a standard orientation (quaternion)
     */
    virtual base::Orientation orientation() const = 0;

     /** normalized main_confidence for this particle */
     double main_confidence;
};


/**
 * Abstract class for a particle filter based on the theory of Thrun, Burgard and Fox
 * already provides importance resampling with a low variance sampler and uses a given estimation 
 * model for processing each particle representation
 */
template <typename P, typename U, typename M>
class ParticleFilter : Dynamic<P,U> {
  public:
    ParticleFilter() 
    {}

    virtual ~ParticleFilter() {}

   typedef typename std::vector<P>::iterator ParticleIterator;

   /**
    * initialize this filter with new particle samples 
    */
   virtual void initialize(int numbers,
           const Eigen::Vector3d& pos, const Eigen::Vector3d& pos_covariance,
           double yaw, double yaw_covariance) = 0; 

    /**
     * create a position sample from the the given particle set. Assure
     * you are getting only a valid position after a measurement update
     * (after dynamics the mean is outdated again)
     *
     * Implementation can override this method for choosing  average value
     */
    virtual base::samples::RigidBodyState& estimate() = 0;

    /**
     * updates the current particle set for an incoming motion action depending
     * on the given estimation model.
     *
     * \param motion representation of a motion call
     */
    virtual void update(const U& motion) {
        // brutal hack and performance could suffer a little, but it works
        Dynamic<P, U>* model = dynamic_cast<Dynamic<P, U>*>(this);

        unsigned best_particle = 0;
        unsigned i = 0;
	base::Position mean = base::Position::Zero();
	base::Matrix3d variance = base::Matrix3d::Zero();

	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	    model->dynamic(*it, motion);
	    mean += it->position();

            if(it->main_confidence > particles[best_particle].main_confidence) {
                best_particle = i;
            }

            i++;
	}

	mean_position = mean / particles.size();
        best_position = particles[best_particle].position();

	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	    base::Vector3d pos_s = it->position() - mean;
            variance += pos_s * pos_s.transpose();
	}

	cov_position = variance / (particles.size() + 1);

	timestamp = model->getTimestamp(motion);
    }

    template<typename Z>
    double observe(const Z& z, const M& m, double ratio = 1.0)
    {
        Perception<P, Z, M>* model = dynamic_cast<Perception<P, Z, M>*>(this);

        std::vector<double> perception_weights;
        double sum_perception_weight = 0.0;
        double sum_main_confidence = 0.0;
        double Neff = 0.0;
        unsigned i;

        // calculate all perceptions
        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            perception_weights.push_back(model->perception(*it, z, m));
            sum_perception_weight += perception_weights.back();
        }

        if(sum_perception_weight <= 0.0) 
            return 0.0;

        i = 0;

        // normalize perception weights and form mixed weight based on probabilities of perception, random_noise, maximum_range_noise
        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            double uniform = 1.0 / particles.size();
            double w = 1.0 - ratio;

            it->main_confidence *= ((ratio * perception_weights[i++] / sum_perception_weight) 
                + w * uniform);

            sum_main_confidence += it->main_confidence;
        }

        // normalize overall confidence
        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            it->main_confidence = it->main_confidence / sum_main_confidence;

            Neff += it->main_confidence * it->main_confidence;
        }

        return (1.0 / Neff) / particles.size();
    }

    /** 
     * returns current status of particle set in a general form
     *
     * \return filled particle vector
     */
    const ParticleSet& getParticleSet() { 
        assert(particles.size() > 0);

	ps.particles.clear();

	ps.timestamp = timestamp;
        ps.weights = weights;

        ps.best_particle = 0;
        unsigned i = 0;
	
	for(ParticleIterator it = particles.begin(); it != particles.end(); it++, i++) {
            Particle p;
	    p.position =it->position();
	    p.yaw = base::getYaw(it->orientation());
	    p.main_confidence = it->main_confidence;

            if( !std::isnan(it->main_confidence) 
                    && particles[ps.best_particle].main_confidence < it->main_confidence)
                ps.best_particle = i;

	    ps.particles.push_back(p);
            ps.generation = generation;
	}

        return ps; 
    }

    /**
     * execute an importance resampling process on the current particle set
     * with an low variance sampler introduced by Thrun, Burgard and Fox 
     */
    void resample() {
          if(particles.size() < 1)
              return;

          double m_inv = 1.0 / particles.size();
          machine_learning::UniformRealRandom random = machine_learning::Random::uniform_real(0.0, m_inv);

          std::vector<P> set;
          double r = random();
          double c = particles.front().main_confidence;
          
          for(unsigned i = 0, m = 0; m < particles.size(); m++) {
              double u = r + (m * m_inv);
              while(u > c) {
                  if(++i >= particles.size())
                      i = 0;

                  c += particles[i].main_confidence;
              }

              set.push_back(particles[i]);
          }

          generation++;

          particles = set;
      }

  protected:
      /** current using particle set of state hypothesis */
      std::vector<P> particles;

      base::Time timestamp;

      unsigned int generation;

      base::Vector3d weights;

      base::Position best_position;
      base::Position mean_position;
      base::Matrix3d cov_position;

      ParticleSet ps;
};


}

#endif // UW_LOCALIZATION__PARTICLEFILTER_H_
