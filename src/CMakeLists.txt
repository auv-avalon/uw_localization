set(CPP_FILES
    math/geometry.cpp
    filters/particle_filter.cpp
    maps/map.cpp
    maps/node_map.cpp
    maps/grid_map.cpp
    maps/depth_obstacle_grid.cpp
    model/uw_motion_model.cpp
    #test/graph_test.cpp
    #dp_slam/dp_slam.cpp
    #dp_slam/node_tree.cpp
    )

set(HPP_FILES
    math/geometry.hpp
    filters/particle_filter.hpp
    types/particle.hpp
    types/map.hpp
    types/environment.hpp
    types/info.hpp
    maps/map.hpp
    maps/node_map.hpp
    maps/grid_map.hpp
    maps/depth_obstacle_grid.hpp
    model/uw_motion_model.hpp
    #dp_slam/dp_slam.hpp
    #dp_slam/node_tree.hpp
    )

rock_library(uw_localization
    SOURCES ${CPP_FILES}
    HEADERS ${HPP_FILES}
    DEPS_PKGCONFIG yaml-cpp
    DEPS_PKGCONFIG machine_learning 
    DEPS_PKGCONFIG base-types 
    DEPS_PKGCONFIG eigen3)
    
#rock_executable(graph_test
#    SOURCES test/graph_test.cpp
#    DEPS_PKGCONFIG uw_localization
#    DEPS_PKGCONFIG libunicap
#    DEPS_PKGCONFIG opencv base-lib
#)    

