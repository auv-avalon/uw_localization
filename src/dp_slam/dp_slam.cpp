#include "dp_slam.hpp"

using namespace uw_localization;




DPSlam::~DPSlam(){
 
  
}

double DPSlam::getDepth( double x, double y, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return NAN;
  
  GridCell elem = get(ID.x(), ID.y());
  
  Feature f = getFeature(elem, id, true);
  
  if(f.id != id || f.depth_confidence <= 0.0){
    return NAN;    
  }
  
  return f.depth;
  
}

int64_t DPSlam::setDepth(double x, double y, double depth, double confidence, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return NAN;
  
  GridCell elem = get( ID.x(), ID.y());
  
  //Initial set. return nothing
  if(std::isnan(elem.pos.x()) || confidence == 0.0){
    elem.pos = base::Vector2d(x,y);
    //set(ID.x(), ID.y(), elem);
    return 0;
  }
  
  Feature f = getFeature(elem, id);
  
  //There is no feature for this cell
  if(f.id == 0){
    id = getNewID();
    
    f.id = id;
    f.depth = depth;
    f.depth_confidence = confidence;
    f.used = true;
    elem.features.push_back(f);
    return id;    
  }
  
  if(f.depth_confidence < confidence){
    id = getNewID();
    
    f.id = id;
    f.depth = depth;
    f.depth_confidence = confidence;
    f.used = true;
    elem.features.push_back(f);
    return id;
    
  }
  
  return id;

}

bool DPSlam::getObstacle( double x, double y, int64_t id){

  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0 )
    return false;
  
  GridCell elem = get(ID.x(), ID.y());
  
  Feature f = getFeature(elem, id, true);
  
  //No feature for this cell/id
  if(f.id != id || f.obstacle_confidence <= 0.0)
    return false;
  
  return f.obstacle;  
  
}

int64_t DPSlam::setObstacle(double x, double y, bool obstacle, double confidence, int64_t id){

  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return id;
  
  GridCell elem = get(ID.x(), ID.y());
  
  Feature f = getFeature(elem, id);
  
  //No feature for this cell. Create one
  if(f.id == 0){
    id = getNewID();
    Feature feature;
    feature.obstacle = obstacle;
    feature.obstacle_confidence = confidence;
    feature.id = id;
    feature.used = true;
    elem.features.push_back(feature);
    return id;
  }
  
  //There is already a feature, but with less confidence
  if(f.obstacle_confidence < confidence){
    id = getNewID();
    Feature feature;
    feature.obstacle = obstacle;
    feature.obstacle_confidence = confidence;
    feature.id = id;
    feature.used = true;
    elem.features.push_back(feature);
    return id;  
    
  }
  
  //There is alredy a better feature
  return id;

}

Feature DPSlam::getFeature(GridCell &cell, int64_t id, bool flag){
 
  for(std::list<Feature>::iterator it = cell.features.begin(); it != cell.features.end(); it++){
    
    if(it->id == id){
      
      if(flag){
        it->used = true;
      }
      
      return *it;
    
    }
      
  }
  
  return Feature();
  
}


void DPSlam::setFeature(GridCell &cell, int64_t id, Feature feature){
  
  for(std::list<Feature>::iterator it = cell.features.begin(); it != cell.features.end(); it++){
    
    if(it->id == id){
      *it = feature;
    }
    
  }
  
  
}


base::samples::Pointcloud DPSlam::getCloud(std::list<std::pair<Eigen::Vector2i,int64_t > > cells){
  
  base::samples::Pointcloud result;
   
  for(std::list<std::pair<Eigen::Vector2i,int64_t > >::iterator it = cells.begin(); it != cells.end(); it++){
      
      GridCell cell = get(it->first.x(), it->first.y());
    
      for(std::list<Feature>::iterator it_features = cell.features.begin(); it_features != cell.features.end(); it_features++){
    
        if(it_features->id != it->second)
          continue;
        
        if(it_features->depth_confidence > 0.0){
      
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->depth);
          result.points.push_back(vec);
          result.colors.push_back(base::Vector4d(it_features->depth_confidence, it_features->depth_confidence, it_features->depth_confidence, 1.0  ) );
        }
        
        if(it_features->obstacle_confidence > 0.0 && it_features->obstacle){
          
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->obstacle_confidence);
          result.points.push_back(vec);
          result.colors.push_back(base::Vector4d(1.0, 0.0, 0.0, 1.0 ) );        
        }
      }
    
  }
  
  return result;
}

int64_t DPSlam::getNewID(){
  lastID++;
  return lastID;
  
}
