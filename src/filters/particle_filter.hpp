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
    virtual double perception(P& state, const Z& sensor, const M& map) = 0;
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
    virtual double dynamic(P& state, const U& motion) = 0;
};




/**
 * Abstract class for a particle filter based on the theory of Thrun, Burgard and Fox
 * already provides importance resampling with a low variance sampler and uses a given estimation 
 * model for processing each particle representation
 */
template <typename P>
class ParticleFilter {
    typedef typename std::vector<P>::iterator StateIterator;
  protected:
    ParticleFilter() {}
    virtual ~ParticleFilter() {}
    
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
     * get unnormlized weight for this current state particle 
     * 
     * \param state of type P
     * \return current weight for this state 
     */
    virtual double getWeight(const P& state) const = 0;


    /**
     * set unnormalized weight for this state particle
     *
     * \param state of type P
     * \param new weight
     */
    virtual void setWeight(P& state, double value) = 0;
 
    
    
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
    template<typename U>
    void update(const P& state, const U& motion) {
        // brutal hack and performance could suffer a little, but it works
        Dynamic<P, U>* model = dynamic_cast<Dynamic<P, U>*>(this);

        updateParticleSet(boost::bind(&Dynamic<P, U>::dynamic, model, _1, motion));
    }


    /**
     * updates the current particle set for an incoming sensor sample depending on the 
     * given estimation model
     *
     * \param sensor measurement sample
     * \param map current map for this particle
     * \return global propability current particle set is not in kidnapping problem
     */
    template<typename Z, typename M>
    void observe(const Z& sensor, const M& map) {
        // brutal hack and performance could suffer a little, but it works
        Perception<P, Z, M>* model = dynamic_cast<Perception<P, Z, M>*>(this);

        updateParticleSet(boost::bind(&Perception<P, Z, M>::perception, model, _1, sensor, map));
    }



    void updateParticleSet(const boost::function1<double, P&>& update)
    {
        unsigned best_particle_index = 0;
        double sum_weight = 0.0;
        double confidence = 0.0;
        double max_weight = 0.0;

        particle_set.particles.clear();

        base::Position mean = base::Position::Zero();
        base::Matrix3d variance = base::Matrix3d::Zero();

        unsigned index = 0;

        for(StateIterator it = states.begin(); it != states.end(); it++) {
            confidence += update(*it);
            sum_weight += getWeight(*it);

            if(getWeight(*it) > max_weight)
                best_particle_index = index++;
            else
                index++;

            mean += position(*it);
        }

        mean_position = (mean / states.size());

        particle_set.particles.clear();

        for(StateIterator it = states.begin(); it != states.end(); it++) {
            Particle p;
            p.position = position(*it);
            p.yaw = base::getYaw(orientation(*it));
            p.norm_weight = getWeight(*it) / sum_weight;

            // calculate covariance for this set
            base::Vector3d pos_s = position(*it) - mean;
            variance += pos_s * pos_s.transpose();

            particle_set.particles.push_back(p);
        }

        cov_position = (variance / (states.size() + 1));

        particle_set.confidence = (confidence / states.size());
        particle_set.max_particle_index = best_particle_index;
    }


    /** 
     * returns current status of particle set in a general form
     * can only accessed if not a resampling is applied before
     *
     * \return filled particle vector
     */
    const ParticleSet& getParticleSet() { 
        assert(particle_set.particles.size() > 0);

        return particle_set; 
    }

    /**
     * execute an importance resampling process on the current particle set
     * with an low variance sampler introduced by Thrun, Burgard and Fox 
     */
    void resample() {
          if(particle_set.particles.size() < 1)
              return;

          double m_inv = 1.0 / states.size();
          machine_learning::UniformRandom random = machine_learning::Random::uniform(0.0, m_inv);

          const std::vector<Particle>& p = particle_set.particles;
          
          std::vector<P> set;
          double r = random();
          double c = p.front().norm_weight;

          for(unsigned i = 0, m = 0; m < p.size(); m++) {
              double u = r + (m * m_inv);
              while(u > c) {
                  assert(i < p.size());
                  c += p[++i].norm_weight;
              }

              set.push_back(states[i]);

              set_weight(set.back()) = 1.0 / p.size();
          }

          // calculate directly means for position and orientation
          states = set;

          particle_set.particles.clear();
      }

  protected:
      /** current using particle set of state hypothesis */
      std::vector<P> states;
      ParticleSet particle_set;

      base::Position mean_position;
      base::Matrix3d cov_position;
};


}

#endif // UW_LOCALIZATION__PARTICLEFILTER_H_
