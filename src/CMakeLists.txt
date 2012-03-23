set(CPP_FILES
    math/geometry.cpp
    filters/particle_filter.cpp
    )

set(HPP_FILES
    math/geometry.hpp
    filters/particle_filter.hpp
    types/particle.hpp
    )

rock_library(uw_localization
    SOURCES ${CPP_FILES}
    HEADERS ${HPP_FILES}
    DEPS_PKGCONFIG base-types
    DEPS_PKGCONFIG yaml-cpp
    DEPS_PKGCONFIG machine_learning)

