#include "dp_map.hpp"

using namespace uw_localization;




DPMap::~DPMap(){ 
  
}


void DPMap::initalize_statics(NodeMap *map){
  
  Environment env = map->getEnvironment();
  
  for(std::vector<Plane>::iterator it = env.planes.begin(); it != env.planes.end(); it++){
    
    double plane_length = Eigen::Vector2d( it->span_horizontal.x(), it->span_horizontal.y() ).norm();
    Eigen::Vector2d step = Eigen::Vector2d( it->span_horizontal.x(), it->span_horizontal.y() ) / (plane_length / (0.5 * resolution) );
    double step_length = step.norm();
        
    Eigen::Vector2d lastCell(NAN, NAN);
    Eigen::Vector2d pos = Eigen::Vector2d(it->position.x(), it->position.y());
    
    //iterate through the cells
    for(double i = 0.0; i < plane_length; i += step_length){
      
      Eigen::Vector2d cell_coord = getGridCoord( pos.x(), pos.y() );
      
      //step was not wide enough, we are still in the same cell
      if(cell_coord != lastCell){       
      
        lastCell = cell_coord;
        
        Eigen::Vector2i ID = getCellID(cell_coord.x(), cell_coord.y());
        
        //Step was invalid, we are outside the grid
        if(ID.x() == -1){
          pos += step;
          continue;
        }
        
        //Set the cell static
        GridCell &elem = get(ID.x(), ID.y());      
        elem.is_static = true;
                
      }
      
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
  
  if(f.id != id || f.depth_variance <= 0.0){
    return NAN;    
  }
  
  return f.depth;
  
}

int64_t DPMap::setDepth(double x, double y, double depth, double variance, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return 0;
  
  GridCell& elem = get( ID.x(), ID.y());
  
  //Initial set. return nothing
  if(std::isnan(elem.pos.x()) || variance == 0.0){
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
    f.depth_variance = variance;
    f.used = true;
    elem.features.push_back(f);
    return id;    
  }
  else if(variance > 0.0){
    id = getNewID();
    
    f.id = id;
    
    double k = variance / (variance + f.depth_variance);
    
    f.depth = f.depth + ( k * (depth - f.depth ) ) ;
    f.depth_variance = f.depth_variance - (k * variance);
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
    feature.obstacle_confidence = confidence * 0.5;
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
        
    if(obstacle == false && f.obstacle && confidence > 0){ //we do not see an old feature --> reduce feature confidence
      //feature.obstacle_confidence = ((1.0 - confidence) + f.obstacle_confidence) * 0.5;
      
      //Confidence, that the cell is empty
      double confidence_empty = (1.0 - f.obstacle_confidence) + confidence - ((1.0 - f.obstacle_confidence) * confidence);
      
      //Obstacle ceonfidence is reverse of empty-confidence
      feature.obstacle_confidence = 1.0 - confidence_empty;
      
    }else if(obstacle == f.obstacle && obstacle && confidence > 0){ // We recognized the feature before, update the confidence 
      //feature.obstacle_confidence = (confidence + f.obstacle_confidence) * 0.5 ;
      feature.obstacle_confidence = f.obstacle_confidence + confidence - (f.obstacle_confidence * confidence);
    }
    else if(obstacle && f.obstacle == false && confidence > 0){ //we recognize this feature for the first time
      feature.obstacle = true;
      
       //The confidence before was 0.5, because we did not know, if the cell is empty or occupied
      feature.obstacle_confidence = confidence + 0.5 - (confidence * 0.5);
    }
      
    if(f.obstacle_confidence <= 0.0){ // We have no confidence in this feature
      f.obstacle = false;
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
                                          std::list<std::pair<Eigen::Vector2d,int64_t > > &obstacle_cells,
                                          double confidence_threshold){

  base::samples::Pointcloud result;
  
  //Create depth output
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = depth_cells.begin(); it != depth_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;
      
      GridCell cell = get(ID.x(), ID.y());
    
      for(std::list<Feature>::iterator it_features = cell.features.begin(); it_features != cell.features.end(); it_features++){
    
        if(it_features->id != it->second)
          continue;
        
        if(it_features->depth_variance > 0.0){
      
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->depth);
          result.points.push_back(vec);
          result.colors.push_back(base::Vector4d(1.0, 1.0, 1.0, 1.0  ) );
        }
        
      }
    
  }
  int i = 0;
  //Create obstacle output
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = obstacle_cells.begin(); it != obstacle_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;    
    
      GridCell cell = get(ID.x(), ID.y()); 
    
      
      for(std::list<Feature>::iterator it_features = cell.features.begin(); it_features != cell.features.end(); it_features++){
    
        if(it_features->id != it->second)
          continue;
        

        if(confidence_threshold < 1.0 && it_features->obstacle_confidence > confidence_threshold && it_features->obstacle){
          
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->obstacle_confidence);
          result.points.push_back(vec);
          
          double sat = (it_features->obstacle_confidence - confidence_threshold) / (1.0 - confidence_threshold) ;
          
          if(sat > 1.0)
            sat = 1.0;
          
          result.colors.push_back(base::Vector4d(sat, 0.0, 0.0 , 1.0 ) );        
        }else{
          i++;
        }
        
      }
    
  }
  //std::cout << "Filtered " << i << std::endl;
  //Search for static cells
  for(std::vector<GridCell>::iterator it = grid.begin(); it != grid.end(); it++){
    
    if(it->is_static){
      
      result.points.push_back( base::Vector3d(it->pos.x(), it->pos.y(), 0.0) );
      result.colors.push_back(base::Vector4d(0.0, 1.0, 0.0, 1.0) );
      
    }
    
  }
  
  return result;
}

int64_t DPMap::getNewID(){
  lastID++;
  return lastID;  
}

void DPMap::reduceFeatures(double confidence_threshold){

  //Iterate through grid cells
  for(std::vector<GridCell>::iterator it = grid.begin(); it!= grid.end(); it++){
    
    //Iterate through features per cell
    for(std::list<Feature>::iterator it_f = it->features.begin(); it_f != it->features.end(); ){
      
      //Delete features, if they were unused or if obstacle confidence is below threshold
      if( ((!it_f->used) &&  (!isnan(it_f->depth) ) ) || (it_f->obstacle && it_f->obstacle_confidence < confidence_threshold) )
      {
        it_f = it->features.erase(it_f);
                
        if(it_f == it->features.end())
           break;
        
      }
      else{
        it_f->used = false;
        ++it_f;
      }
      
    }
    
  }

}

std::list< std::pair<Eigen::Vector2d, double > > DPMap::getObservedCells(std::vector<Eigen::Vector2d> &cells, std::list<std::pair<Eigen::Vector2d,int64_t > > &ids){
  
  std::list< std::pair<Eigen::Vector2d, double > > result;
  
  //Search through cell list, and find static cells or observed cells
  for(std::vector<Eigen::Vector2d>::iterator it = cells.begin(); it != cells.end(); it++){
    
    Eigen::Vector2i ID = getCellID(it->x(), it->y());
      
    //Is cell id valid?
    if(ID.x() < 0)
      continue;    
         
    GridCell &cell = get(ID.x(), ID.y());
     
    //Cell is static, we dont need to search for observations
    if(cell.is_static){
      result.push_back( std::make_pair(*it, 1.0) );
      continue;
    }
        
    //Search for coresponding observation
    for(std::list<std::pair<Eigen::Vector2d, int64_t > >::iterator it_id = ids.begin(); it_id != ids.end(); it_id++){
      
      //Chec id      
      if(*it != it_id->first)
        continue;     

      Feature f = getFeature(cell, it_id->second, true);
        
      if(f.id != 0 && f.obstacle && f.obstacle_confidence > 0.0){
        result.push_back( std::make_pair(*it, f.obstacle_confidence ) );

      }
      
      
    }    
    
  }
  
  return result;
}


