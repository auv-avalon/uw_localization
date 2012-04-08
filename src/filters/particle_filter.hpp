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
    virtual double perception(const P& state, const Z& sensor, const M& map) {
	throw new std::runtime_error("perception model is required");
	return 0.0;
    }


    virtual bool isMaximumRange(const Z& sensor) {
	return false;
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

    virtual const base::Time& getTimestamp(const U& motion) = 0;
};




/**
 * Abstract class for a particle filter based on the theory of Thrun, Burgard and Fox
 * already provides importance resampling with a low variance sampler and uses a given estimation 
 * model for processing each particle representation
 */
template <typename P, typename U, typename M>
class ParticleFilter : Dynamic<P,U> {
    typedef typename std::vector<P>::iterator ParticleIterator;
  public:
    ParticleFilter() 
    {}

    virtual ~ParticleFilter() {}

   /**
    * initialize this filter with new particle samples 
    */
   virtual void initialize(int numbers,
           const Eigen::Vector3d& pos, const Eigen::Matrix3d& pos_covariance,
           double yaw, double yaw_covariance) = 0; 

   /**
     * get a general position representation from a given abstract pose particles
     *
     * \param state of type P
     * \return a simple vector with three dimension
     */
    virtual base::Vector3d position(const P& state) const = 0;

    /**
     * get a general orientation representation from a given abstract poste particle
     * \param state of type P
     * \return a standard orientation (quaternion)
     */
    virtual base::Orientation orientation(const P& state) const = 0;


    /**
     * get normlized weight for this current state particle 
     * 
     * \param state of type P
     * \return current weight for this state 
     */
    virtual double getWeight(const P& state) const = 0;



    /**
     * set normalized weight for this state particle
     *
     * \param state of type P
     * \param new weight
     */
    virtual void setWeight(P& state, double value) = 0;

    /**
     * checks if a current state particle is still valid in a map
     */
    virtual bool belongsToWorld(const P& state, const M& map) { 
       return true;
    }
   
    
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

	base::Position mean = base::Position::Zero();
	base::Matrix3d variance = base::Matrix3d::Zero();

	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	    model->dynamic(*it, motion);
	    mean += position(*it);	
	}

	mean_position = mean / particles.size();

	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	    base::Vector3d pos_s = position(*it) - mean;
            variance += pos_s * pos_s.transpose();
	}

	cov_position = variance / (particles.size() + 1);

	timestamp = model->getTimestamp(motion);
    }


    /**
     * updates the current particle set for an incoming sensor sample depending on the 
     * given estimation model
     *
     * \param sensor measurement sample
     * \param map current map for this particle
     * \return global propability current particle set is not in kidnapping problem
     */
    template<typename Z>
    double observe(const Z& sensor, const M& map, const Eigen::Vector3d& ratio = Eigen::Vector3d(1.0, 0.0, 0.0), double random_noise = -1.0) {
        // brutal hack and performance could suffer a little, but it works
        Perception<P, Z, M>* model = dynamic_cast<Perception<P, Z, M>*>(this);

	std::vector<double> quick_weights;
        Eigen::Vector3d weights;
	double sum_perception_weight = 0.0;
	double sum_overall_weight = 0.0;
        double overall_weight = 0.0;
        double Neff = 0.0;
	unsigned i;

	best_particle = 0;

        if(random_noise < 0)
	     random_noise = 1.0 / particles.size();

	// calculate all perceptions
	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            if(belongsToWorld(*it, map)) {
	        quick_weights.push_back(model->perception(*it, sensor, map));
	        sum_perception_weight += quick_weights.back();
	    } else {
                quick_weights.push_back(0.0);
	    }
	}

        i = 0;

	// normalize perception weights and form mixed weight based on probabilities of perception, random_noise, maximum_range_noise
	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	    if(model->isMaximumRange(sensor))
	       weights = Eigen::Vector3d(0.0, 1.0, 0.0);
	    else
	       weights = Eigen::Vector3d(quick_weights[i] * sum_perception_weight, 0.0, random_noise);

	    overall_weight = (weights.transpose() * ratio * getWeight(*it)).x();
	    sum_overall_weight += overall_weight;
	    quick_weights[i] = overall_weight;

 	    if(overall_weight > quick_weights[best_particle])
		best_particle = i;

            i++;
	}

        i = 0;

	// normalize overall weights and form effective sample size
	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	   double norm_weight = quick_weights[i] / sum_overall_weight;
	   Neff += norm_weight * norm_weight;

           setWeight(*it, norm_weight);

           i++;
	}

	return 1.0 / Neff;        
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
        ps.max_particle_index = best_particle;
	
	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	    Particle p;
	    p.position = position(*it);
	    p.yaw = base::getYaw(orientation(*it));
	    p.norm_weight = getWeight(*it);

	    ps.particles.push_back(p);
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
          double c = getWeight(particles.front());

          for(unsigned i = 0, m = 0; m < particles.size(); m++) {
              double u = r + (m * m_inv);
              while(u > c) {
                  assert(i < particles.size());
                  c += getWeight(particles[++i]);
              }

	      if(i == best_particle)
		i = set.size() - 1;

              set.push_back(particles[i]);
          }

          particles = set;
      }

  protected:
      /** current using particle set of state hypothesis */
      std::vector<P> particles;

      base::Time timestamp;

      unsigned best_particle;

      base::Position mean_position;
      base::Matrix3d cov_position;

      ParticleSet ps;
};


}

#endif // UW_LOCALIZATION__PARTICLEFILTER_H_
