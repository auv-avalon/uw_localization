/* ----------------------------------------------------------------------------
 * particle_filter.h
 * written by Christoph Mueller, Oct 2011
 * University of Bremen
 * ----------------------------------------------------------------------------
*/
#ifndef UW_LOCALIZATION__PARTICLEFILTER_H_
#define UW_LOCALIZATION__PARTICLEFILTER_H_

#include <vector>
#include <list>
#include <assert.h>
#include <machine_learning/RandomNumbers.hpp>
#include <base/samples/rigid_body_state.h>
#include <uw_localization/types/particle.hpp>

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
    virtual double perception(const P& p, const Z& sensor, M& map) {
	throw new std::runtime_error("perception model is required");
	return 0.0;
    }
};




template<typename P, typename U, typename M>
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
    virtual void dynamic(P& state, const U& motion, const M& map) = 0;

    /**
     * get current timestamp for this motion command
     */
    virtual const base::Time& getTimestamp(const U& motion) = 0;
};



template<typename P>
bool compare_particles(const P& x, const P& y) {
    return x.main_confidence > y.main_confidence;
}


/**
 * Abstract class for a particle filter based on the theory of Thrun, Burgard and Fox
 * already provides importance resampling with a low variance sampler and uses a given estimation 
 * model for processing each particle representation
 */
template <typename P>
class ParticleFilter {
  public:
    ParticleFilter() : generation(0), first_perception_received(true)
    {}

    virtual ~ParticleFilter() {}

   typedef typename std::list<P>::iterator ParticleIterator;

   /**
    * reduce the particle set to ratio percent of the best particles
    *
    * \param ratio (0.0 <= ratio <= 1.0) how many particles are saved after reducing
    * \return how many number of particles are removed
    */
   virtual size_t reduceParticles(double ratio = 0.0) {
       size_t removing = floor(particles.size() * (1.0 - ratio));

       particles.sort(compare_particles<P>);

       for(size_t i = 0; i < removing && particles.size() > 0; i++) {
           particles.pop_back();
       }

       return removing;
   }

   /**
    * normalize the weights for all particles
    */
   virtual void normalizeParticles() {
       double sum = 0.0;
       double neff = 0.0;
       base::Position mean_pos = base::Position::Zero();
       
       for(ParticleIterator it = particles.begin(); it != particles.end(); ++it) {
           mean_pos += position(*it);
           sum += confidence(*it);
       }

       if(sum == 0.0 && particles.size() != 0){
         sum = particles.size();
         
          for(ParticleIterator it = particles.begin(); it != particles.end(); ++it) {
            setConfidence(*it, 0.0);
            neff += confidence(*it) * confidence(*it);
          }
       }
       else{              
       
          for(ParticleIterator it = particles.begin(); it != particles.end(); ++it) {
              setConfidence(*it, confidence(*it) / sum);
              neff += confidence(*it) * confidence(*it);
          }
       }

       mean_position = mean_pos / particles.size();
       effective_sample_size = (1.0 / neff) / particles.size();
   }


   /**
     * get a general position representation from a given abstract pose particles
     *
     * \param state of type P
     * \return a simple vector with three dimension
     */
    virtual base::Position position(const P& X) const = 0;

    /**
     * get a general velocity representation from a given abstract pose particles
     *
     * \param state of type P
     * \return a simple vector with three dimension
     */
    virtual base::Vector3d velocity(const P& X) const = 0;


    /**
     * get a general orientation representation from a given abstract poste particle
     * \param state of type P
     * \return a standard orientation (quaternion)
     */
    virtual base::samples::RigidBodyState orientation(const P& X) const = 0;

    
    /**
     * checks, if a particle is valid, and should be used for position estimation
     * @param state of type P
     * @return true, if particle is valid
     */
    virtual bool isValid(const P& X) const = 0;
    
    /**
     *
     */
    virtual double confidence(const P& X) const = 0;


    virtual void setConfidence(P& X, double weight) = 0;

    /**
     * returns the state vector for the best particle as RigidBodyState
     */
    virtual base::samples::RigidBodyState& estimate() {
        base::Matrix3d variance = base::Matrix3d::Zero();

        ParticleIterator best = particles.begin();

        for(ParticleIterator it = particles.begin(); it != particles.end(); ++it) {
            base::Vector3d pos_s = position(*it) - mean_position;
            variance += pos_s * pos_s.transpose();

            if( confidence(*best) < confidence(*it) )
                best = it;
        }

        state = orientation(*best);

        state.position = position(*best);
        state.cov_position = variance / (particles.size() + 1);

        return state;
    }
    
    /**
     * return the state for the weighted avegrage particle as RBS
     */
    virtual base::samples::RigidBodyState& estimate_middle() {
        base::Matrix3d variance = base::Matrix3d::Zero();

        ParticleIterator best = particles.begin();
        base::Vector3d best_pos(0.0, 0.0, 0.0);
        double sum_conf = 0.0;

        for(ParticleIterator it = particles.begin(); it != particles.end(); ++it) {
            
            if(!isValid(*it))
              continue;
          
            base::Vector3d pos_s = position(*it) - mean_position;
            variance += pos_s * pos_s.transpose();
            
            sum_conf += confidence(*it);
            best_pos += confidence(*it) * position(*it);
        }

        state = orientation(*best);
        
        if(sum_conf > 0){
          state.position = best_pos / sum_conf;
          state.cov_position = variance / (particles.size() + 1);
        }
        else{
          state.position = base::Vector3d(0.0, 0.0, 0.0);
          state.cov_position = base::Matrix3d::Identity() * INFINITY;
        }
          
        return state;
    }    
    

    /**
     * updates the current particle set for an incoming motion action depending
     * on the given estimation model.
     *
     * \param motion representation of a motion call
     */
    template<typename U, typename M>
    void update(const U& motion, const M& map) {

        // brutal hack and performance could suffer a little, but it works
        Dynamic<P, U, M>* model = dynamic_cast<Dynamic<P, U, M>*>(this);

	base::Position mean = base::Position::Zero();

	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
	    model->dynamic(*it, motion, map);
	    mean += position(*it);
	}

	mean_position = mean / particles.size();
	timestamp = model->getTimestamp(motion);
    }

    template<typename Z, typename M>
    double observe(const Z& z, M& m, double ratio = 1.0)
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

        // normalize perception weights and form mixed weight based on current weight.
        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            double uniform = 1.0 / particles.size();
            double w = 1.0 - ratio;
                   
            double main_confidence = confidence(*it) * ((ratio * (perception_weights[i++] / sum_perception_weight)) 
                + w * uniform);

            setConfidence(*it, main_confidence);

            sum_main_confidence += main_confidence;
        }

        // normalize overall confidence
        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            double weight;
            
            if(sum_main_confidence == 0.0)
              weight = 0.0;
            else              
              weight = confidence(*it) / sum_main_confidence;
            
            setConfidence(*it, weight);

            Neff += weight * weight;
        }
        
        if(Neff == 0.0)
          effective_sample_size = 1.0;
        else
          effective_sample_size = (1.0 / Neff) / particles.size();

        return effective_sample_size;
    }

    template<typename Z, typename M>
    double observe_markov(const Z& z, M& m, double ratio = 1.0)
    {
        Perception<P, Z, M>* model = dynamic_cast<Perception<P, Z, M>*>(this);

        double Neff = 0.0;
        double weight;
        double sum = 0.0;

        // calculate all perceptions
        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            double conf = model->perception(*it, z, m);
            setConfidence(*it, conf);
            sum += conf;
        }
        
        // normalize overall confidence
        for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            
            if(sum == 0.0)
              weight = 1 / particles.size();
            else              
              weight = confidence(*it) / sum;
            
            setConfidence(*it, weight);

            Neff += weight * weight;
        }
        
        if(Neff == 0.0)
          effective_sample_size = 1.0;
        else
          effective_sample_size = (1.0 / Neff) / particles.size();

        return effective_sample_size;
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

	for(ParticleIterator it = particles.begin(); it != particles.end(); it++) {
            Particle p;
	    p.position = position(*it);
            p.velocity = velocity(*it);
	    p.yaw = base::getYaw(orientation(*it).orientation);
	    p.main_confidence = confidence(*it);

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

          std::list<P> set;
          double r = random();
          double c = confidence(particles.front());

          ParticleIterator it = particles.begin();
          
          for(unsigned i = 0, m = 0; m < particles.size(); m++) {
              double u = r + (m * m_inv);
              while(u > c) {
                  it++;

                  if(++i >= particles.size()) {
                      it = particles.begin();
                      i = 0;
                  }

                  c += confidence(*it);
              }

              set.push_back(*it);
          }

          normalizeParticles();

          generation++;
          //effective_sample_size = (1.0 / neff) / particles.size();

          particles = set;
      }
  
  protected:
      std::list<P> particles;

      base::Time timestamp;

      double effective_sample_size;
      unsigned int generation;
      bool first_perception_received;

      base::Position mean_position;

      ParticleSet ps;
      base::samples::RigidBodyState state;
};


}

#endif // UW_LOCALIZATION__PARTICLEFILTER_H_
