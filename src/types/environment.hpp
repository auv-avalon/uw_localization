#ifndef UW_LOCALIZATION_TYPES_ENVIRONMENT_HPP
#define UW_LOCALIZATION_TYPES_ENVIRONMENT_HPP

#include <base/Eigen.hpp>
#include <vector>

namespace uw_localization {


struct Plane {
    base::Vector3d position;
    base::Vector3d span_horizontal;
    base::Vector3d span_vertical;
};


struct Landmark {
    std::string caption;
    base::Vector3d point;
};


struct Box{
    base::Vector3d position; //Center of the box
    base::Vector3d span; //length of the object in x,y,z  
};  

struct Environment {
    base::Vector3d left_top_corner;
    base::Vector3d right_bottom_corner;

    std::vector<Plane> planes;
    std::vector<Landmark> landmarks;
};

}

#endif
