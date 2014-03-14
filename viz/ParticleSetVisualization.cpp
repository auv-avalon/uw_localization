#include "ParticleSetVisualization.hpp"
#include <osg/Geode>

namespace vizkit3d
{

ParticleSetVisualization::ParticleSetVisualization()
{
    updated = false;
    default_feature_color = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
}

/**
 * Creates the main node and attachs the point cloud.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > ParticleSetVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
    //pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_PRIMITIVE_SET);
    //pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    
    return mainNode;
}

QColor ParticleSetVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void ParticleSetVisualization::setDefaultFeatureColor(QColor color)
{
    default_feature_color.x() = color.redF();
    default_feature_color.y() = color.greenF();
    default_feature_color.z() = color.blueF();
    default_feature_color.w() = color.alphaF();
    emit propertyChanged("defaultFeatureColor");
}

/**
 * Updates the class with the new particle data of type ParticleSet.
 * 
 * @param data new particleset
 */
void ParticleSetVisualization::updateDataIntern(const uw_localization::ParticleSet& data)
{
    particleSet = data;
    updated = true;
}

void ParticleSetVisualization::updateDataIntern(const std::vector< base::Vector3d >& data)
{
    channelInfos = data;
}

/**
 * Main callback method of osg to update all drawings.
 * The method updates the position if there is a new one and
 * adds new sonar beams if there are new ones. If a wall 
 * estimatior is connected the position of the wall will 
 * be updated too.
 * 
 * @param node osg main node
 */


void ParticleSetVisualization::updateMainNode(osg::Node* node)
{   

    if(updated)
    {
        updated = false;
        pointsOSG->clear();
        color->clear();

	double min = 0.0, max = 1.0 / particleSet.particles.size();
	
	for(std::vector<uw_localization::Particle>::const_iterator it = particleSet.particles.begin(); it != particleSet.particles.end(); it++){
	  if(it->main_confidence > max){
	    max = it->main_confidence;
	  }
	}
	
	
	
	for(std::vector<uw_localization::Particle>::const_iterator it = particleSet.particles.begin(); it != particleSet.particles.end(); it++)
        {
	  
	  pointsOSG->push_back(osg::Vec3(it->position[0], it->position[1], 0.0));
	  pointsOSG->push_back(osg::Vec3(it->position[0], it->position[1], (it->main_confidence / max )  ));
	  osg::Vec4f newColor( hsv2rgb(it->main_confidence / max ,1,1 ) );	
	  color->push_back(newColor);
	}	
	
	drawArrays->setCount(pointsOSG->size());
	pointGeom->setVertexArray(pointsOSG);
	pointGeom->setColorArray(color);
    }

}

// http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb
osg::Vec4f ParticleSetVisualization::hsv2rgb(double h, double s, double v){
    
    osg::Vec4f newColor;
    newColor.w() = 255;
    double      hh, p, q, t, ff;
    long        i;

    if(s <= 0.0) {       // < is bogus, just shuts up warnings
        if(isnan(h)) {   // in.h == NAN
            newColor.x() = v;
            newColor.y() = v;
            newColor.z() = v;
            return newColor;
        }
        // error - should never happen
        newColor.x() = 0.0;
        newColor.y() = 0.0;
        newColor.z() = 0.0;
        return newColor;
    }
    hh = h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = v * (1.0 - s);
    q = v * (1.0 - (s * ff));
    t = v * (1.0 - (s * (1.0 - ff)));

    switch(i) {
    case 0:
        newColor.x() = v;
        newColor.y() = t;
        newColor.z() = p;
        break;
    case 1:
        newColor.x() = q;
        newColor.y() = v;
        newColor.z() = p;
        break;
    case 2:
        newColor.x() = p;
        newColor.y() = v;
        newColor.z() = t;
        break;

    case 3:
        newColor.x() = p;
        newColor.y() = q;
        newColor.z() = v;
        break;
    case 4:
        newColor.x() = t;
        newColor.y() = p;
        newColor.z() = v;
        break;
    case 5:
    default:
        newColor.x() = v;
        newColor.y() = p;
        newColor.z() = q;
        break;
    }
    newColor.x() *= 255;
    newColor.y() *= 255;
    newColor.z() *= 255;
    return newColor; 
  
  
}  

}
