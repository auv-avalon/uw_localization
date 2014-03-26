Vizkit::UiLoader.register_3d_plugin('ParticleSetVisualization', 'uw_localization', 'ParticleSetVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ParticleSetVisualization', "uw_localization/ParticleSet", :updateParticleSet )

Vizkit::UiLoader.register_3d_plugin('ParticleVisualization', 'uw_localization', 'ParticleVisualization')
Vizkit::UiLoader.register_3d_plugin_for('ParticleVisualization', "uw_localization/ParticleSet", :updateParticleSet)

Vizkit::UiLoader.register_3d_plugin('MapVisualization', 'uw_localization', 'MapVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MapVisualization', 'uw_localization/Environment', :updateEnv)

Vizkit::UiLoader.register_3d_plugin('MonitorVisualization', 'uw_localization', 'MonitorVisualization')
Vizkit::UiLoader.register_3d_plugin_for('MonitorVisualization', 'uw_localization/Environment', :updateEnv)
Vizkit::UiLoader.register_3d_plugin_for('MonitorVisualization', 'uw_localization/ParticleSet', :updateParticleSet)
Vizkit::UiLoader.register_3d_plugin_for('MonitorVisualization', 'uw_localization/ParticleInfo', :updateInfo)

Vizkit::UiLoader.register_3d_plugin('SonarPointVisualization', 'uw_localization', 'SonarPointVisualization')
Vizkit::UiLoader.register_3d_plugin_for('SonarPointVisualization', 'uw_localization/PointInfo', :updateInfo)
