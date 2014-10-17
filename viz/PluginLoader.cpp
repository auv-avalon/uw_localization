#include <vizkit3d/Vizkit3DPlugin.hpp>
#include "ParticleSetVisualization.hpp"
#include "ParticleVisualization.hpp"
#include "MapVisualization.hpp"
#include "MonitorVisualization.hpp"
#include "SonarPointVisualization.hpp"
#include "SimpleGridVisualization.hpp"
#include <iostream>

namespace uw_localization {
    class QtPluginVizkit : public vizkit3d::VizkitPluginFactory {
    private:
    public:
	
	QtPluginVizkit() {
	}
	
	/**
	* Returns a list of all available visualization plugins.
	* @return list of plugin names
	*/
        virtual QStringList* getAvailablePlugins() const
	{
            QStringList *pluginNames = new QStringList();
            pluginNames->push_back("ParticleSetVisualization");
            pluginNames->push_back("ParticleVisualization");
            pluginNames->push_back("MapVisualization");
            pluginNames->push_back("MonitorVisualization");
            pluginNames->push_back("SonarPointVisualization");
            pluginNames->push_back("SimpleGridVisualization");

	    return pluginNames;
	}
	
        virtual QObject* createPlugin(QString const& pluginName)
        {
	    vizkit3d::VizPluginBase* plugin = 0;
	    if (pluginName == "ParticleSetVisualization")
	    {
		plugin = new vizkit3d::ParticleSetVisualization();
	    }
	    else if(pluginName == "ParticleVisualization")
	    {
	      plugin = new vizkit3d::ParticleVisualization();
	    }
	    else if(pluginName == "MapVisualization")
	    {
	      plugin = new vizkit3d::MapVisualization();
	    }
	    else if(pluginName == "MonitorVisualization")
	    {
	      plugin = new vizkit3d::MonitorVisualization();
	    }
	    else if(pluginName == "SonarPointVisualization")
	    {
	      plugin = new vizkit3d::SonarPointVisualization();
	    }
	    else if(pluginName == "SimpleGridVisualization")
           {
              plugin = new vizkit3d::SimpleGridVisualization();
           }
	    else{
	      std::cout << "No pluginmatch" << std::endl;
	    }
	      
	    if (plugin) 
	    {
		return plugin;
	    }
	    return NULL;
        };
    };
    Q_EXPORT_PLUGIN2(QtPluginVizkit, QtPluginVizkit)
}
