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


struct Linemark {
    Linemark() : from(0.0, 0.0, 0.0), to(1.0, 0.0, 0.0), height(1.0) {}

    /** current start point for this line **/
    base::Vector3d from;

    /** current end point for this line **/
    base::Vector3d to;

    /** height of this line in world **/
    double height;
};


struct MixedMap {
    /** current world limitations **/
    base::Vector3d limitations;

    /** translation for limitations to reference frame **/
    base::Vector3d translation;

    /** landmarks for this map **/
    std::vector<uw_localization::Landmark> landmarks;

    /** wall lines in this map **/
    std::vector<uw_localization::Linemark> lines;
};


}

#endif 
