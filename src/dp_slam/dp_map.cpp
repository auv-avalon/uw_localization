#include "dp_map.hpp"

using namespace uw_localization;




DPMap::~DPMap(){ 
  
}


void DPMap::initalizeStatics(NodeMap *map){
  
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

bool DPMap::initalizeStaticDepth(const std::string &filename){
    std::ifstream fin(filename.c_str());
    if(fin.fail()) {
        std::cerr << "Could not open input stream from file" << filename.c_str() << std::endl;
        return false;
    }
    
    YAML::Parser parser(fin);
    YAML::Node doc;
    Eigen::Vector3d vec;

    while(parser.GetNextDocument(doc)) {
      
      for(unsigned i=0;i<doc.size();i++) {
      
        doc[i] >> vec;
        setStaticDepth(vec.x(), vec.y(), vec.z(), 0.0);        
        
      }
    }

}

void DPMap::initDepthObstacleConfig(double min_depth, double max_depth, double depth_resolution){
  
  this->max_depth = max_depth;
  this->min_depth = min_depth;
  this->depth_resolution = depth_resolution;
  
}


double DPMap::getDepth( double x, double y, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return NAN;
  
  GridCell& elem = get(ID.x(), ID.y());
  
  if(!std::isnan(elem.static_depth)){
    return elem.static_depth;
  }
  
  DepthFeature f = getDepthFeature(elem, id, true);
  
  if(f.id != id || f.depth_variance <= 0.0){
    return NAN;    
  }
  
  return f.depth;
  
}

void DPMap::setStaticDepth(double x, double y, double depth, double variance){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return;
  
  GridCell& elem = get( ID.x(), ID.y());
  
  if( std::isinf(elem.static_depth_variance)){
  
    elem.static_depth = depth;
    elem.static_depth_variance = variance;
  }
  else{
    double k = elem.static_depth_variance / (variance + elem.static_depth_variance);
    
    elem.static_depth = elem.static_depth + ( k * (depth - elem.static_depth ) ) ;
    elem.static_depth_variance = elem.static_depth_variance - (k * elem.static_depth_variance);
    
  }
    
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
  
  DepthFeature f = getDepthFeature(elem, id, true);
  
  //There is no feature for this cell
  if(id == 0 || f.id == 0 || std::isinf(f.depth_variance) ){
    id = getNewID();
    
    f.id = id;
    f.depth = depth;
    f.depth_variance = variance;
    f.used = true;
    elem.depth_features.push_back(f);
    return id;    
    //std::cout << "Set new depth: " << depth << " var: " << f.depth_variance << std::endl;
  }
  else if(variance > 0.0){
    id = getNewID();
    
    f.id = id;
    
    double k = f.depth_variance / (variance + f.depth_variance);
    
    double temp_depth = f.depth;
    f.depth = f.depth + ( k * (depth - f.depth ) ) ;
    f.depth_variance = f.depth_variance - (k * f.depth_variance);
    
    //std::cout << "Correct depth: " << depth << " - old depth: " << temp_depth << " - new depth: " << f.depth << " k: " << k << " depth_variance: " << f.depth_variance << std::endl;
    
    f.used = true;
    elem.depth_features.push_back(f);
    return id;
    
  }
  
  return 0;

}

bool DPMap::getObstacle( double x, double y, int64_t id){

  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0 )
    return false;
  
  GridCell& elem = get(ID.x(), ID.y());
  
  if(elem.is_static)
    return true;
  
  ObstacleFeature f = getObstacleFeature(elem, id, true);
  
  //No feature for this cell/id
  if(f.id != id || f.obstacle_confidence <= 0.0 )
    return false;
  
  return f.obstacle;  
  
}

int64_t DPMap::setObstacle(double x, double y, bool obstacle, double confidence, double min_depth, double max_depth, int64_t id){

  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return 0;
  
  GridCell& elem = get(ID.x(), ID.y());
  
//  if(elem.is_static)
//    return 0;
  
  ObstacleFeature f = getObstacleFeature(elem, id, true);
  
  //No feature for this cell. Create one
  // 
  if(f.id == 0){
    
    if(obstacle){    
      id = getNewID();
      ObstacleFeature feature;
      feature.init_confidences(min_depth, max_depth, depth_resolution);
      feature.obstacle = true;
      feature.obstacle_confidence = confidence + 0.5 - (confidence * 0.5);
      setObstacleDepthConfidence(feature.obstacle_depth_confidence, min_depth, max_depth, confidence, obstacle);
      feature.obstacle_count++;
      feature.id = id;
      feature.used = true;
      //std::cout << "Set new feature, x: " << x << " y: " << y << " id: " << id << std::endl;
      elem.obstacle_features.push_back(feature);
      return id;
    }
    else{  //Do not set initial empty cells!
      return 0;
    }
    
  }
  
  //There is already a feature, but with less confidence
  if(confidence > 0){
    id = getNewID();
    ObstacleFeature feature = f;
    
    setObstacleDepthConfidence(feature.obstacle_depth_confidence, min_depth, max_depth, confidence, obstacle);
        
    if(obstacle == false && f.obstacle && confidence > 0){ //we do not see an old feature --> reduce feature confidence
      //feature.obstacle_confidence = ((1.0 - confidence) + f.obstacle_confidence) * 0.5;
      
      //Confidence, that the cell is empty
      double confidence_empty = (1.0 - f.obstacle_confidence) + confidence - ((1.0 - f.obstacle_confidence) * confidence);
      
      //Obstacle ceonfidence is reverse of empty-confidence
      feature.obstacle_confidence = 1.0 - confidence_empty;
      
    }else if(obstacle == f.obstacle && obstacle && confidence > 0){ // We recognized the feature before, update the confidence 
      //feature.obstacle_confidence = (confidence + f.obstacle_confidence) * 0.5 ;
      feature.obstacle_confidence = f.obstacle_confidence + confidence - (f.obstacle_confidence * confidence);
      feature.obstacle_count++;
    }
    else if(obstacle && f.obstacle == false && confidence > 0){ //we recognize this feature for the first time
      feature.obstacle = true;
      
       //The confidence before was 0.5, because we did not know, if the cell is empty or occupied
      feature.obstacle_confidence = confidence + 0.5 - (confidence * 0.5);
      feature.obstacle_count++;
    }
      
    if(feature.obstacle_confidence <= 0.0 && !feature.is_obstacle(0.0000001)){ // We have no confidence in this feature
      feature.obstacle = false; 
    }
      
    feature.id = id;
    feature.used = true;
    //std::cout << "Set feature again, x: " << x << " y: " << y << " id: " << id << std::endl;
    elem.obstacle_features.push_back(feature);
    return id;  
    
  }
  
  //std::cout << "There is an old, better feature" << std::endl;
  //There is alredy a better feature
  return 0;
}

void DPMap::touchDepthFeature(double x, double y, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return;
  
  GridCell& elem = get(ID.x(), ID.y());
    
  getDepthFeature(elem, id, true);
  
}

void DPMap::touchObstacleFeature(double x, double y, int64_t id){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return;
  
  GridCell& elem = get(ID.x(), ID.y());
    
  getObstacleFeature(elem, id, true);
  
}



DepthFeature DPMap::getDepthFeature(GridCell &cell, int64_t id, bool flag){
 
  for(std::list<DepthFeature>::iterator it = cell.depth_features.begin(); it != cell.depth_features.end(); it++){
    
    if(it->id == id){
      
      if(flag){
        it->used = true;
      }
      
      return *it;
    
    }
      
  }
  
  return DepthFeature();
  
}

ObstacleFeature DPMap::getObstacleFeature(GridCell &cell, int64_t id, bool flag){
 
  for(std::list<ObstacleFeature>::iterator it = cell.obstacle_features.begin(); it != cell.obstacle_features.end(); it++){
    
    if(it->id == id){
      
      if(flag){
        it->used = true;
      }
      
      return *it;
    
    }
      
  }
  
  return ObstacleFeature();
  
}



void DPMap::setDepthFeature(GridCell &cell, int64_t id, DepthFeature &feature){
  
  for(std::list<DepthFeature>::iterator it = cell.depth_features.begin(); it != cell.depth_features.end(); it++){
    
    if(it->id == id){
      *it = feature;
    }
    
  }  
  
}

void DPMap::setObstacleFeature(GridCell &cell, int64_t id, ObstacleFeature &feature){
  
  for(std::list<ObstacleFeature>::iterator it = cell.obstacle_features.begin(); it != cell.obstacle_features.end(); it++){
    
    if(it->id == id){
      *it = feature;
    }
    
  }  
  
}


void DPMap::setObstacleDepthConfidence(std::vector<ObstacleDepthConfidence> &depth_vector, double min, double max, double confidence, bool obstacle){
  
  for(std::vector<ObstacleDepthConfidence>::iterator it = depth_vector.begin(); it != depth_vector.end(); it++){
    
    //Is this observation grid inside our observation?
    if( max >= it->lower_border && min < it->upper_border){
      
      if(obstacle == false && it->obstacle && confidence > 0){ //we do not see an old feature --> reduce feature confidence
        
        //Confidence, that the cell is empty
        double confidence_empty = (1.0 - it->confidence) + confidence - ((1.0 - it->confidence) * confidence);
        
        //Obstacle ceonfidence is reverse of empty-confidence
        it->confidence = 1.0 - confidence_empty;
        
      }else if(obstacle == it->obstacle && obstacle && confidence > 0){ // We recognized the feature before, update the confidence 

        it->confidence = it->confidence + confidence - (it->confidence * confidence);
        
      }
      else if(obstacle && it->obstacle == false && confidence > 0){ //we recognize this feature for the first time
        it->obstacle = true;
        
        //The confidence before was 0.5, because we did not know, if the cell is empty or occupied
        it->confidence = confidence + 0.5 - (confidence * 0.5);
        
      }
      
      if(it->obstacle && it->confidence <= 0.0){
        //it->obstacle = false; TODO re-add
      }      
      
    }    
    
  }  
  
}


base::samples::Pointcloud DPMap::getCloud(std::list<std::pair<Eigen::Vector2d,int64_t > > &depth_cells,
                                          std::list<std::pair<Eigen::Vector2d,int64_t > > &obstacle_cells,
                                          double confidence_threshold, int count_threshold){

  base::samples::Pointcloud result;
  result.colors.clear();
  result.points.clear();
  
  //std::cout << "Get cloud, obstacles: " << obstacle_cells.size() << std::endl;
  //std::cout << "Get cloud, depth: " << depth_cells.size() << std::endl;
  int i = 0;
  int var_i = 0;
  //Create depth output
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = depth_cells.begin(); it != depth_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;
      
      GridCell cell = get(ID.x(), ID.y());
          
      for(std::list<DepthFeature>::iterator it_features = cell.depth_features.begin(); it_features != cell.depth_features.end(); it_features++){
    
        if(it_features->id != it->second)
          continue;
        
        i++;
        
        if(it_features->depth_variance > 0.0){
          
          var_i++;
          
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->depth);
          result.points.push_back(vec);
          result.colors.push_back(base::Vector4d(1.0, 1.0, 1.0, 1.0  ) );
        }
        
      }
    
  }
  
  //std::cout << "Depth: " << i << " , var low: " << var_i - i << std::endl;
  
  i = 0;
  int below_confidence_threshold = 0;
  int no_obstacle_count = 0;
  int below_depth_obstacle_threshold = 0;
  int below_count_threshold = 0;
  
  //Create obstacle output
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = obstacle_cells.begin(); it != obstacle_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;    
    
      GridCell cell = get(ID.x(), ID.y()); 
    
      //Search for features with correspoding id
      for(std::list<ObstacleFeature>::iterator it_features = cell.obstacle_features.begin(); it_features != cell.obstacle_features.end(); it_features++){
    
        //Wrong id
        if(it_features->id != it->second){
          continue;
        }
          
        //Use feature, if it is an obstacle and has a strong confidence or was obsered many times
        if(confidence_threshold < 1.0 && it_features->obstacle   && 
          ( it_features->obstacle_confidence > confidence_threshold || it_features->obstacle_count > count_threshold || it_features->is_obstacle(confidence_threshold) ) ){
          
          base::Vector3d vec(cell.pos.x(), cell.pos.y(), it_features->obstacle_confidence);
          result.points.push_back(vec);
          
          double sat = (it_features->obstacle_confidence - confidence_threshold) / (1.0 - confidence_threshold) ;
          
          if(sat > 1.0){
            sat = 1.0;
          
          }else if(sat < 0.0){
            sat = 0.0;
          }
          
          if( it_features->obstacle_count > count_threshold){
            result.colors.push_back(base::Vector4d(0.0, 0.0, 1.0, 1.0) );
          }
          else{
            result.colors.push_back(base::Vector4d(sat, 0.0, 0.0 , 1.0 ) );
          }
        }else{
          i++;
          
          if(!it_features->obstacle)
            no_obstacle_count++;
          
          if(it_features->obstacle_confidence <= confidence_threshold)
            below_confidence_threshold++;
          
          if(it_features->obstacle_count <= count_threshold)
            below_count_threshold++;
          
          if(!it_features->is_obstacle(confidence_threshold))
            below_depth_obstacle_threshold++;
          
        }
        
      }
    
  }

  for(std::vector<GridCell>::iterator it = grid.begin(); it != grid.end(); it++){
    
    if(it->is_static){
      
      result.points.push_back( base::Vector3d(it->pos.x(), it->pos.y(), 0.0) );
      result.colors.push_back(base::Vector4d(0.0, 1.0, 0.0, 1.0) );
      
    }
    
    if( !std::isinf(it->static_depth_variance) ){
      
          base::Vector3d vec(it->pos.x(), it->pos.y(), it->static_depth );
          result.points.push_back(vec);
          result.colors.push_back(base::Vector4d(1.0, 1.0, 1.0, 1.0  ) );      
      
    }
    
  }
  
  return result;
}

void DPMap::getSimpleGrid(uw_localization::SimpleGrid &simple_grid, std::list<std::pair<Eigen::Vector2d,int64_t > > &depth_cells,
                                          std::list<std::pair<Eigen::Vector2d,int64_t > > &obstacle_cells,
                                          double confidence_threshold, int count_threshold){

  simple_grid.init(position, span, resolution);
  

  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = depth_cells.begin(); it != depth_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;
      
      GridCell cell = get(ID.x(), ID.y());
          
      for(std::list<DepthFeature>::iterator it_features = cell.depth_features.begin(); it_features != cell.depth_features.end(); it_features++){
    
        if(it_features->id != it->second)
          continue;
        
        
        if(it_features->depth_variance > 0.0){

          SimpleGridElement elem;
          
          simple_grid.getCell(it->first.x(), it->first.y(), elem);
          
          elem.depth = it_features->depth;
          
          simple_grid.setCell(it->first.x(), it->first.y(), elem);
          

        }
        
      }    
  }

  
  //Create obstacle output
  for(std::list<std::pair<Eigen::Vector2d,int64_t > >::iterator it = obstacle_cells.begin(); it != obstacle_cells.end(); it++){
      
      Eigen::Vector2i ID = getCellID(it->first.x(), it->first.y());
      
      if(ID.x() < 0)
        continue;    
    
      GridCell cell = get(ID.x(), ID.y()); 
    
      //Search for features with correspoding id
      for(std::list<ObstacleFeature>::iterator it_features = cell.obstacle_features.begin(); it_features != cell.obstacle_features.end(); it_features++){
    
        //Wrong id
        if(it_features->id != it->second){
          continue;
        }
          
        //Use feature, if it is an obstacle and has a strong confidence or was obsered many times
        if(confidence_threshold < 1.0 && it_features->obstacle   && 
          ( it_features->obstacle_confidence > confidence_threshold || it_features->obstacle_count > count_threshold || it_features->is_obstacle(confidence_threshold) ) ){

          SimpleGridElement elem;
          simple_grid.getCell(it->first.x(), it->first.y(), elem);
        
          elem.obstacle = true;
        
          if(it_features->obstacle_count < count_threshold){
            elem.obstacle_conf = it_features->obstacle_confidence;
          }
          else{
            elem.obstacle_conf = 1.0;
          }
           
          simple_grid.setCell(it->first.x(), it->first.y(), elem); 

        }
        
      }
    
  }

  for(std::vector<GridCell>::iterator it = grid.begin(); it != grid.end(); it++){
    
    
    if(it->is_static){
      
      uw_localization::SimpleGridElement elem;
      
      simple_grid.getCell(it->pos.x(), it->pos.y(), elem);
      
      elem.obstacle = true;
      elem.static_object = true;
      elem.obstacle_conf = NAN;
      
      simple_grid.setCell(it->pos.x(), it->pos.y(), elem);
      
    }
    
    if( !std::isinf(it->static_depth_variance) ){
      
        uw_localization::SimpleGridElement elem;
      
        simple_grid.getCell(it->pos.x(), it->pos.y(), elem);
        
        elem.depth = it->static_depth;
        
        simple_grid.setCell(it->pos.x(), it->pos.y(), elem);     
      
    }
    
  }
  
}


int64_t DPMap::getNewID(){
  lastID++;
  return lastID;  
}

void DPMap::reduceFeatures(double confidence_threshold, int count_threshold){

  //Iterate through grid cells
  for(std::vector<GridCell>::iterator it = grid.begin(); it!= grid.end(); it++){
    
    //Iterate through features per cell
    for(std::list<ObstacleFeature>::iterator it_f = it->obstacle_features.begin(); it_f != it->obstacle_features.end(); ){
      
      //Delete features, if they were unused or if obstacle confidence is below threshold
      if( (!it_f->used) ||
        (it_f->obstacle && it_f->obstacle_confidence < confidence_threshold 
        && it_f->obstacle_count < count_threshold && !it_f->is_obstacle(confidence_threshold)  ) )
      {
        it_f = it->obstacle_features.erase(it_f);
                
        if(it_f == it->obstacle_features.end())
           break;
        
      }
      else{
        it_f->used = false;
        ++it_f;
      }
      
    }
    
    //Iterate through features per cell
    for(std::list<DepthFeature>::iterator it_f = it->depth_features.begin(); it_f != it->depth_features.end(); ){
      
      //Delete features, if they were unused or if obstacle confidence is below threshold
      if( (!it_f->used) ||  isnan(it_f->depth) )
      {
        it_f = it->depth_features.erase(it_f);
                
        if(it_f == it->depth_features.end())
           break;
        
      }
      else{
        //it_f->used = false;
        
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
      
      //Check id      
      if(*it != it_id->first)
        continue;     

      ObstacleFeature f = getObstacleFeature(cell, it_id->second, true);
        
      if(f.id != 0 && f.obstacle && f.obstacle_confidence > 0.0){
        result.push_back( std::make_pair(*it, f.obstacle_confidence ) );

      }
      
      
    }    
    
  }
  
  return result;
}


