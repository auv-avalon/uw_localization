rock_vizkit_plugin(uw_localization_laserscan-viz
   LaserScanVisualization.cpp
   DEPS uw_localization
   MOC LaserScanVisualization.hpp
   HEADERS LaserScanVisualization.hpp)

rock_library(uw_localization_viz
    SOURCES ParticleGeode.cpp
    HEADERS ParticleGeode.hpp
    DEPS_PKGCONFIG vizkit)

rock_vizkit_plugin(uw_localization_particle-viz
    ParticleVisualization.cpp
    DEPS uw_localization uw_localization_viz
    MOC ParticleVisualization.hpp
    HEADERS ParticleVisualization.hpp ParticleGeode.hpp)

rock_vizkit_plugin(uw_localization_sonarpoint-viz
    SonarPointVisualization.cpp
    DEPS uw_localization uw_localization_viz
    MOC SonarPointVisualization.hpp
    HEADERS SonarPointVisualization.hpp ParticleGeode.hpp)

rock_vizkit_plugin(uw_localization_map-viz
   MapVisualization.cpp
   DEPS uw_localization
   MOC MapVisualization.hpp
   HEADERS MapVisualization.hpp)




