#ifndef UW_LOCALIZATION_SAMPLES_INFO_HPP
#define UW_LOCALIZATION_SAMPLES_INFO_HPP

namespace uw_localization {

struct PointInfo {
    /** timestamp for debug information **/
    base::Time time;

   /** desired point for estimation */
    base::Vector3d desire_point;

    /** real point measured */
    base::Vector3d real_point;

    /** status message */
    std::string status;

    /** confidence for this measurement */
    double confidence;

    /** measurement distance */
    double distance;
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
