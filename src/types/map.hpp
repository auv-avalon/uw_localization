/* ----------------------------------------------------------------------------
 * samples/map.h
 * written by Christoph Mueller, Mar 2012
 * University of Bremen
 * ----------------------------------------------------------------------------
*/

#ifndef UW_LOCALIZATION_SAMPLES_MAP_HPP_
#define UW_LOCALIZATION_SAMPLES_MAP_HPP_

#include <base/eigen.h>
#include <string>

namespace uw_localization {

struct Landmark {
    Landmark() : mean(0.0, 0.0, 0.0), covariance(base::Matrix3d::Identity()) {}

    /** current landmark description **/
    std::string caption;

    /** mean position for this landmark **/
    base::Vector3d mean;

    /** covariance for this landmark **/
    base::Matrix3d covariance;
};


struct LandmarkMap {
    /** current world limitations **/
    base::Vector3d limitations;

    /** current landmarks for this map **/
    std::vector<uw_localization::Landmark> landmarks;
};


}

#endif 
