rock_vizkit_plugin(uw_localization_particle-viz
    ParticleVisualization.cpp
    DEPS uw_localization
    MOC ParticleVisualization.hpp
    HEADERS ParticleVisualization.hpp)

rock_vizkit_plugin(uw_localization_laserscan-viz
   LaserScanVisualization.cpp
   DEPS uw_localization
   MOC LaserScanVisualization.hpp
   HEADERS LaserScanVisualization.hpp)

rock_vizkit_plugin(uw_localization_mapfeature-viz
   MapFeatureVisualization.cpp
   DEPS uw_localization
   MOC MapFeatureVisualization.hpp
   HEADERS MapFeatureVisualization.hpp)

rock_library(uw_localization_viz
    SOURCES ParticleGeode.cpp
    HEADERS ParticleGeode.hpp
    DEPS_PKGCONFIG vizkit)

rock_vizkit_plugin(uw_localization_monitor-viz
    MonitorVisualization.cpp
    DEPS uw_localization uw_localization_viz
    MOC MonitorVisualization.hpp
    HEADERS MonitorVisualization.hpp ParticleGeode.hpp)




