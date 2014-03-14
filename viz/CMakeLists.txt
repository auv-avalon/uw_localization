rock_export_includedir(${PROJECT_SOURCE_DIR}/viz vizkit)

rock_vizkit_plugin(uw_localization-viz
   MOC ParticleGeode.cpp PluginLoader.cpp LaserScanVisualization.cpp ParticleVisualization.cpp ParticleSetVisualization.cpp SonarPointVisualization.cpp MapVisualization.cpp MonitorVisualization.cpp
   DEPS uw_localization
   DEPS_PKGCONFIG vizkit3d
   HEADERS ParticleGeode.hpp LaserScanVisualization.hpp ParticleVisualization.hpp ParticleSetVisualization.hpp SonarPointVisualization.hpp MapVisualization.hpp MonitorVisualization.hpp
   )
   


