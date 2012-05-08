#ifndef UW_LOCALIZATION_TYPES_ENVIRONMENT_HPP
#define UW_LOCALIZATION_TYPES_ENVIRONMENT_HPP

namespace uw_localization {

#include <base/eigen.h>

struct Plane {
    base::Vector3d position;
    base::Vector3d span_dim1;
    base::Vector3d span_dim2;
};


struct Environment {
    base::Vector3d left_top_corner;
    base::Vector3d right_bottom_corner;

    std::vector<Plane> planes;
};

}

#endif
