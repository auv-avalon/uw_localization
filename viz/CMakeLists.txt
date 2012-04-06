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

rock_vizkit_plugin(uw_localization_mixedmap-viz
   MixedMapVisualization.cpp
   DEPS uw_localization
   MOC MixedMapVisualization.hpp
   HEADERS MixedMapVisualization.hpp)



