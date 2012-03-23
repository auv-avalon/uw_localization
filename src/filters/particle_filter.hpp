/* ----------------------------------------------------------------------------
 * particle_filter.h
 * written by Christoph Mueller, Oct 2011
 * University of Bremen
 * ----------------------------------------------------------------------------
*/
#ifndef UW_LOCALIZATION__PARTICLEFILTER_H_
#define UW_LOCALIZATION__PARTICLEFILTER_H_

#define BASE_LOG_NAMESPACE Particlefilter

#include <vector>
#include <stdio.h>
#include <boost/smart_ptr.hpp>
#include <boost/assert.hpp>
#include <machine_learning/RandomNumbers.hpp>
#include <uw_localization/types/particle.hpp>
#include <base/samples/rigid_body_state.h>
#include <base/logging.h>

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
     * \param state current representation of a state hypothesis (from a single particle)
     * \param map current representation for a given map hypothesis
     * \return probability of plausible measurement zt given we are in state xt and a map m
     */
    virtual double perception(const Z& sensor, const P& state, const M& map) = 0;
};

template<typename P, typename U>
class Dynamic {
 public:
    /**
     * represents the motion model the particle filter use for drawing a sample of the 
     * probability xt[m] ~ P(xt | xt-1, ut). Keep in mind xt[m] is just a sample
     * and must be involved with a gaussian bias (requirement for particle filters)
     *
     * \param state current representation of a state hypothesis (from a single particle)
     * \param motion current representation of a motion call
     * \return a new state sample depending on we were in state xt-1 and we move with ut
     */
    virtual P dynamic(const P& state, const U& motion) = 0;
};


/**
 * Abstract class for a particle filter based on the theory of Thrun, Burgard and Fox
 * already provides importance resampling with a low variance sampler and uses a given estimation 
 * model for processing each particle representation
 */
template <typename P, typename M>
class ParticleFilter {
    typedef typename std::vector<P>::iterator ParticleIterator;
  protected:
    /**
     * initializes the given particle set with an uniform distribution of state hypothesis
     *
     * \param particles current particle set that needs to be initialized
     * \param numbers of particles that should be created
     */
    virtual void initialize(std::vector<P>& particles, const int numbers) = 0;
   
  public:
    ParticleFilter() : overall_weight(1.0), mean_position(0.0, 0.0, 0.0) {}
    ~ParticleFilter() {}

    /**
     * get a general position representation from a given abstract pose particles
     *
     * \param state of type P
     * \return a simple vector with three dimension
     */
    virtual base::Vector3d position(P state) const = 0;

    /**
     * get a general orientation representation from a gvien abstract pose particles
     *
     * \ param state of type P
     * \return a standard orientation
     */
    virtual base::Orientation orientation(P state) const = 0;
    
    /**
     * updates the current particle set for an incoming motion action depending
     * on the given estimation model.
     *
     * \param motion representation of a motion call
     */
    template<typename U>
    void update(const U& motion) {
        // brutal hack and performance could suffer a little, but it works
        Dynamic<P, U>* model = dynamic_cast<Dynamic<P, U>*>(this);

        std::vector<P> set;

        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            set.push_back(model->dynamic(*it, motion));
        }

        particles = set;
    }

    /**
     * updates the current particle set for an incoming sensor sample depending on the 
     * given estimation model
     *
     * \param sensor measurement sample
     * \param map current map for this particle
     */
    template<typename Z>
    void observe(const Z& sensor, const M& map) {
        // brutal hack and performance could suffer a little, but it works
        Perception<P, Z, M>* model = dynamic_cast<Perception<P, Z, M>*>(this);

        overall_weight = 0.0;

        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            it->confidence += model->perception(sensor, *it, map);
            overall_weight += it->confidence;
        }
        measure_updates++;
    }

    /**
     * returns the current status of the internal particle set
     * \return current particle set
     */
    inline const std::vector<P>& getParticleSet() const {
        return particles;
    }

    /** 
     * returns current status of particle set in a general form
     * \return filled particle vector
     */
    virtual void getParticles(std::vector<uw_localization::Particle>& particles) const = 0;

    /**
     * create a position sample from the the given particle set. Assure
     * you are getting only a valid position after a measurement update
     * (after dynamics the mean is outdated again)
     */
    virtual base::samples::RigidBodyState estimatePosition() const {
        base::samples::RigidBodyState state;

        state.time = base::Time::now();

        state.position = mean_position;
        state.orientation = mean_orientation;
        state.velocity = base::Vector3d(0.0, 0.0, 0.0);
        state.angular_velocity = base::Vector3d(0.0, 0.0, 0.0);

        base::Matrix3d variance = base::Matrix3d::Zero();

        for(unsigned i = 0; i < particles.size(); i++) {
            base::Vector3d pos_s = particles[i].position - mean_position;
            variance += pos_s * pos_s.transpose();
        }        

        state.cov_position = variance / (particles.size() + 1);

        state.cov_orientation = base::Matrix3d::Zero();
        state.cov_velocity = base::Matrix3d::Zero();
        state.cov_angular_velocity = base::Matrix3d::Zero();

        return state;
    }

    /**
     * get number of measurement updates after a resampling step
     */
    unsigned getMeasureUpdates() const {
        return measure_updates;
    }

    /**
     * execute an importance resampling process on the current particle set
     * with an low variance sampler introduced by Thrun, Burgard and Fox 
     */
    void resample() {
          if(measure_updates == 0)
              return;

          double m_inv = (overall_weight / measure_updates) / particles.size();
          machine_learning::UniformRandom random = machine_learning::Random::uniform(0.0, m_inv);
          
          std::vector<P> set;
          double r = random();
          double c = particles.front().confidence / measure_updates;

          overall_weight = 0;
          mean_position = base::Vector3d(0.0, 0.0, 0.0);

          for(unsigned i = 0, m = 0; m < particles.size(); m++) {
              double u = r + (m * m_inv);
              while(u > c) {
                  BOOST_ASSERT(i < particles.size());
                  c += particles[++i].confidence / measure_updates;
              }

              mean_position += position(particles[i]);
              //overall_weight += (particles[i].confidence / measure_updates);
              
              // assumes fixed orientation
              mean_orientation = orientation(particles[i]);

              set.push_back(particles[i]);
              set.back().confidence = 1.0 / particles.size();
          }

          // calculate directly means for position and orientation
          mean_position /= particles.size();
          particles = set;

          measure_updates = 0;
      }

  protected:
      /** current using particle set of state hypothesis */
      std::vector<P> particles;
      double overall_weight;
      unsigned measure_updates;
      
      base::Vector3d mean_position;
      base::Orientation mean_orientation;
};



};

#endif // UW_LOCALIZATION__PARTICLEFILTER_H_
