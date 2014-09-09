#include "SimpleGridVisualization.hpp"
#include <osg/Geode>

namespace vizkit3d
{

SimpleGridVisualization::SimpleGridVisualization()
{
    updated = false;
    default_feature_color = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f);
}

/**
 * Creates the main node and attachs the point cloud.
 * 
 * @return main node
 */
osg::ref_ptr< osg::Node > SimpleGridVisualization::createMainNode()
{
    osg::ref_ptr<osg::Group> mainNode = new osg::Group();
    
    // set up point cloud
    pointGeom = new osg::Geometry;
    pointsOSG = new osg::Vec3Array;
    pointGeom->setVertexArray(pointsOSG);
    color = new osg::Vec4Array;
    pointGeom->setColorArray(color);
    pointGeom->setColorBinding(osg::Geometry::BIND_PER_VERTEX); //PRIMITIVE_SET);
    pointGeom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF); 
    drawArrays = new osg::DrawArrays( osg::PrimitiveSet::LINES, 0, pointsOSG->size() );
    pointGeom->addPrimitiveSet(drawArrays.get());

    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(pointGeom.get());
    mainNode->addChild(geode);
    
    return mainNode;

}

QColor SimpleGridVisualization::getDefaultFeatureColor()
{
    QColor color;
    color.setRgbF(default_feature_color.x(), default_feature_color.y(), default_feature_color.z(), default_feature_color.w());
    return color;
}

void SimpleGridVisualization::setDefaultFeatureColor(QColor color)
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
void SimpleGridVisualization::updateDataIntern(const uw_localization::SimpleGrid& data)
{
    grid = data;
    updated = true;
}

void SimpleGridVisualization::updateDataIntern(const std::vector< base::Vector3d >& data)
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


void SimpleGridVisualization::updateMainNode(osg::Node* node)
{   

    if(updated)
    {
      
      updated = false;
      pointsOSG->clear();
      color->clear();      
      
      base::Vector2d origin = grid.origin;
      base::Vector2d span = grid.span;
      double resolution = grid.resolution;
      
      int count_invalid = 0;
      
      //Iterate through cordinates
      for(double x = ( -origin.x() ) ; x < (-origin.x() ) + span.x(); x += resolution){
        
        for(double y = ( -origin.y() ) ; y < (-origin.y() ) + span.y(); y += resolution){
          
          uw_localization::SimpleGridElement elem;
          
          //Get cell for actual coordinates -> check if get is valid
          if(grid.getCell(x, y, elem)){
            
            //If obstacle, draw it
            if(elem.obstacle && elem.obstacle_conf > 0){
              pointsOSG->push_back(osg::Vec3d(x, y, 0.0));
              pointsOSG->push_back(osg::Vec3d(x, y, elem.obstacle_conf));
              
              if(elem.obstacle_conf > 1.0){
                color->push_back(osg::Vec4f(0.0, 0.0, 1.0, 1.0));
                color->push_back(osg::Vec4f(0.0, 0.0, 1.0, 1.0));
              }else{
              
                double val = elem.obstacle_conf;
                
                if(val > 1.0){
                  val = 1.0;
                }
                else if(val < 0.0){
                  val = 0.0;
                }
                
                color->push_back(osg::Vec4f(val, 0.0, 0.0, 1.0 ));
                color->push_back(osg::Vec4f(val, 0.0, 0.0, 1.0 ));
              }              
              
            }else if(elem.obstacle && elem.static_object){
                
                pointsOSG->push_back(osg::Vec3d(x, y, 0.0));
                pointsOSG->push_back(osg::Vec3d(x, y, 2.0));
                
                color->push_back(osg::Vec4f(0.0, 1.0, 0.0, 1.0));
                color->push_back(osg::Vec4f(0.0, 1.0, 0.0, 1.0));
                
            } 
            
                        
            
            if(!std::isnan(elem.depth)){
            
              bool found_right_neighbor = false;
              bool found_upper_neighbor = false;
              uw_localization::SimpleGridElement neighbor_elem;
              double right_depth = elem.depth;
              double upper_depth = elem.depth;
              double right_upper_depth = elem.depth;
              
              //Check right
              if(grid.getCell(x + resolution, y, neighbor_elem )){
                
                //We have a valid right neighbor -> draw a line to neighbor
                if(!std::isnan(neighbor_elem.depth)){
                  right_depth = neighbor_elem.depth;
                  found_right_neighbor = true;
                }        
                
              }
              
              pointsOSG->push_back(osg::Vec3(x, y, elem.depth));
              pointsOSG->push_back(osg::Vec3(x + resolution, y, right_depth));
              color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
              color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
              
              
              //Check up
              if(grid.getCell(x, y + resolution, neighbor_elem )){
                
                //We have a valid upper neighbor -> draw a line to neighbor
                if(!std::isnan(neighbor_elem.depth)){
                  found_upper_neighbor = true;
                  upper_depth = neighbor_elem.depth;

                }        
                
              }
              
              pointsOSG->push_back(osg::Vec3(x, y, elem.depth));
              pointsOSG->push_back(osg::Vec3(x, y + resolution, upper_depth));
              color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
              color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
              
              if( !found_right_neighbor ){
                pointsOSG->push_back(osg::Vec3(x + resolution, y, right_depth ));
                pointsOSG->push_back(osg::Vec3(x + resolution, y + resolution, right_depth));
                color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
                color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
                
              }
              
              if( !found_upper_neighbor ){
                pointsOSG->push_back(osg::Vec3(x, y + resolution, upper_depth));
                pointsOSG->push_back(osg::Vec3(x + resolution, y + resolution, right_depth));
                color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
                color->push_back(osg::Vec4f(1.0, 1.0, 1.0, 1.0));
                
              } 
              
              
            }
            
          }
          else{
            count_invalid++;
          }
          
        }       
        
      }
      
//        std::cout << "PointsOSG size: " << pointsOSG->size() << " color size: " << color->size() << std::endl;
//        std::cout << "Invalid count: " << count_invalid << " from " << grid.grid.size() << std::endl;
        
        drawArrays->setCount(pointsOSG->size());
        pointGeom->setVertexArray(pointsOSG);
        pointGeom->setColorArray(color);
        
    }

}

// http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb
osg::Vec4f SimpleGridVisualization::hsv2rgb(double h, double s, double v){
    
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
