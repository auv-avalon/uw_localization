#include "dp_map.hpp"

using namespace uw_localization;




DPMap::~DPMap(){ 
  
}


void DPMap::initalize_statics(NodeMap *map){
  
  Environment env = map->getEnvironment();
  
  for(std::vector<Plane>::iterator it = env.planes.begin(); it != env.planes.end(); it++){
    
    double plane_length = Eigen::Vector2d( it->span_horizontal.x(), it->span_horizontal.y() ).norm();
    Eigen::Vector2d step = Eigen::Vector2d( it->span_horizontal.x(), it->span_horizontal.y() ) / (plane_length / (2.0 * resolution) );
    double step_length = step.norm();
    
    Eigen::Vector2d lastCell(NAN, NAN);
    Eigen::Vector2d pos = Eigen::Vector2d(it->position.x(), it->position.y());
    
    //iterate through the cells
    for(double i = 0.0; i < plane_length; i += step_length){
      
      Eigen::Vector2d cell_coord = getGridCoord( pos.x(), pos.y() );
      
      //step was not wide enough, we are still in the same cell
      if(cell_coord == lastCell)
        continue;
      
      lastCell = cell_coord;
      
      Eigen::Vector2i ID = getCellID(cell_coord.x(), cell_coord.y());
      
      //Step was invalid, we are outside the grid
      if(ID.x() == -1)
        continue;
      
      //Set the cell static
      GridCell &elem = get(ID.x(), ID.y());      
      elem.is_static = true;
      
      pos += step;
      
    }
    
  }
  
}

double DPMap::getDepth( double x, double y, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return NAN;
  
  GridCell& elem = get(ID.x(), ID.y());
  
  Feature f = getFeature(elem, id, true);
  
  if(f.id != id || f.depth_confidence <= 0.0){
    return NAN;    
  }
  
  return f.depth;
  
}

int64_t DPMap::setDepth(double x, double y, double depth, double confidence, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return 0;
  
  GridCell& elem = get( ID.x(), ID.y());
  
  //Initial set. return nothing
  if(std::isnan(elem.pos.x()) || confidence == 0.0){
    elem.pos = base::Vector2d(x,y);
    
    //set(ID.x(), ID.y(), elem);
    return 0;
  }
  
  Feature f = getFeature(elem, id, true);
  
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

bool DPMap::getObstacle( double x, double y, int64_t id){

  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0 )
    return false;
  
  GridCell& elem = get(ID.x(), ID.y());
  
  if(elem.is_static)
    return true;
  
  Feature f = getFeature(elem, id, true);
  
  //No feature for this cell/id
  if(f.id != id || f.obstacle_confidence <= 0.0)
    return false;
  
  return f.obstacle;  
  
}

int64_t DPMap::setObstacle(double x, double y, bool obstacle, double confidence, int64_t id){

  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return id;
  
  GridCell& elem = get(ID.x(), ID.y());
  
  if(elem.is_static)
    return 0;
  
  Feature f = getFeature(elem, id, true);
  
  //No feature for this cell. Create one
  if(f.id == 0){
    id = getNewID();
    Feature feature;
    feature.obstacle = obstacle;
    feature.obstacle_confidence = confidence;
    feature.id = id;
    feature.used = true;
    //std::cout << "Set new feature, x: " << x << " y: " << y << " id: " << id << std::endl;
    elem.features.push_back(feature);
    return id;
  }
  
  //There is already a feature, but with less confidence
  if(confidence > 0){
    id = getNewID();
    Feature feature;
        
    if(obstacle == false && f.obstacle && confidence > 0){ //reduce feature confidence
      feature.obstacle_confidence = f.obstacle_confidence * (1.0 - confidence);
      
    }else if(obstacle == f.obstacle){
      feature.obstacle_confidence = (confidence + f.obstacle_confidence) * 0.5 ;
    }
    else{
      feature.obstacle = obstacle;
    }
      
    feature.id = id;
    feature.used = true;
    //std::cout << "Set feature again, x: " << x << " y: " << y << " id: " << id << std::endl;
    elem.features.push_back(feature);
    return id;  
    
  }
  
  //std::cout << "There is an old, better feature" << std::endl;
  //There is alredy a better feature
  return id;

}

Feature DPMap::getFeature(GridCell &cell, int64_t id, bool flag){
 
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


void DPMap::setFeature(GridCell &cell, int64_t id, Feature feature){
  
  for(std::list<Feature>::iterator it = cell.features.begin(); it != cell.features.end(); it++){
    
    if(it->id == id){
      *it = feature;
    }
    
  }  
  
}


base::samples::Pointcloud DPMap::getCloud(std::list<std::pair<Eigen::Vector2d,int64_t > > &depth_cells,
                                          std::list<std::pair<Eigen::Vector2d,int64_t > > &obstacle_cells){
  
  base::samples::Pointcloud result;
   
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = depth_cells.begin(); it != depth_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;
      
      GridCell cell = get(ID.x(), ID.y());
    
      for(std::list<Feature>::iterator it_features = cell.features.begin(); it_features != cell.features.end(); it_features++){
    
        if(it_features->id != it->second)
          continue;
        
        if(it_features->depth_confidence > 0.0){
      
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->depth);
          result.points.push_back(vec);
          result.colors.push_back(base::Vector4d(it_features->depth_confidence, it_features->depth_confidence, it_features->depth_confidence, 1.0  ) );
        }
        
      }
    
  }
  std::cout << "Obstacle cells: " << obstacle_cells.size() << std::endl;
  int feature_count = 0;
  int feature_match = 0;
  int valid_cells = 0;
  int zero_confidence_cells = 0;
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = obstacle_cells.begin(); it != obstacle_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;    
    
      valid_cells++;
      GridCell cell = get(ID.x(), ID.y()); 
    
      feature_count += cell.features.size();
      
      for(std::list<Feature>::iterator it_features = cell.features.begin(); it_features != cell.features.end(); it_features++){
    
        if(it_features->id != it->second)
          continue;
        
        feature_match++;
        if(it_features->obstacle_confidence > 0.0 && it_features->obstacle){
          
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->obstacle_confidence);
          result.points.push_back(vec);
          result.colors.push_back(base::Vector4d(1.0, 0.0, 0.0, 1.0 ) );        
        }
        else{
          zero_confidence_cells++; 
        }
        
      }
    
  }
  
  //Search for static cells
  for(std::vector<GridCell>::iterator it = grid.begin(); it != grid.end(); it++){
    
    if(it->is_static){
      
      result.points.push_back( base::Vector3d(it->pos.x(), it->pos.y(), 0.0) );
      result.colors.push_back(base::Vector4d(0.0, 1.0, 0.0, 1.0) );
      
    }
    
  }
  
  
  
  std::cout << "Valid cells: " << valid_cells << " Feature count: " << feature_count << " matches: " << feature_match << " zero_counts: " << zero_confidence_cells << std::endl;
  
  return result;
}

int64_t DPMap::getNewID(){
  lastID++;
  return lastID;  
}

void DPMap::reduceFeatures(){

  for(std::vector<GridCell>::iterator it = grid.begin(); it!= grid.end(); it++){
    
    for(std::list<Feature>::iterator it_f = it->features.begin(); it_f != it->features.end(); it_f++){
      
      if(!(it_f->used)){
        it_f = it->features.erase(it_f);
        
        if(it_f == it->features.end())
           break;
        
      }
      else{
        it_f->used = false;
        
      }
      
    }
    
  }
  
}


