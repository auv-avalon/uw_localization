#ifndef UW_LOCALIZATION_SAMPLES_INFO_HPP
#define UW_LOCALIZATION_SAMPLES_INFO_HPP

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <vector>


namespace uw_localization {

enum PointStatus {
    OUT_OF_RANGE, NOT_IN_WORLD, OKAY, MAP_INVALID, OBSTACLE
};

struct PointInfo {
    /** timestamp for debug information **/
    base::Time time;

   /** desired point for estimation */
    base::Vector3d desire_point;

    /** real point measured */
    base::Vector3d real_point;

    /** scan-angle of the measurement*/
    double angle;
    
    /** particle location */
    base::Vector3d location;

    /** current point status */
    PointStatus status;

    /** confidence for this measurement */
    double confidence;

    /** measurement distance */
    double distance;
    
    /**desired distance */
    double desire_distance;
};

struct ParticleInfo {
    /** ParticleInfo type for a specific sensor */
    unsigned int type;

    /** relating particle generation */
    unsigned int generation;

    /** measurement debug information for each particle **/
    std::vector<PointInfo> infos;
};


}

#endif
