#include "depth_obstacle_grid.hpp"

using namespace uw_localization;


DepthObstacleGrid::~DepthObstacleGrid(){}

 
void DepthObstacleGrid::initDepthObstacleConfig(double min_depth, double max_depth, double depth_resolution){
  
  this->max_depth = max_depth;
  this->min_depth = min_depth;
  this->depth_resolution = depth_resolution;
  
} 


double DepthObstacleGrid::getDepth( double x, double y){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return NAN;
  
  GridElement elem = get(ID.x(), ID.y());
  
  return elem.depth;
  
}

void DepthObstacleGrid::setDepth(double x, double y, double depth, double variance){
  
  Eigen::Vector2i ID = getCellID(x,y);
  //std::cout << "idX: " << idX << " span: " << span.x() << " resolution: " << resolution << " x: " << x << " pos: " << position.x() << std::endl;
  
  if(ID.x() < 0)
    return;
  
  GridElement &elem = get(ID.x(), ID.y());
  
  if(variance == 0.0){
    elem.pos = base::Vector2d(x,y);
  }
  else if( std::isinf(elem.depth_variance)){
    elem.depth = depth;
    elem.depth_variance = variance;
  }
  else{
    double k = elem.depth_variance / (variance + elem.depth_variance);
    
    elem.depth = elem.depth + ( k * (depth - elem.depth ) ) ;
    elem.depth_variance = elem.depth_variance - (k * elem.depth_variance);
    
  }
  
}

bool DepthObstacleGrid::getObstacle( double x, double y){
  
  Eigen::Vector2i ID = getCellID(x,y);
  
  if(ID.x() < 0)
    return false;
  
  GridElement elem = get(ID.x(), ID.y());
  
  if(elem.obstacle_confidence <= 0.0)
    return false;
  else
    return elem.obstacle;
  
}

void DepthObstacleGrid::setObstacle(double x, double y, bool obstacle, double confidence){

  Eigen::Vector2i ID = getCellID(x,y);
  //std::cout << "idX: " << idX << " span: " << span.x() << " resolution: " << resolution << " x: " << x << " pos: " << position.x() << std::endl;
  
  if(ID.x() < 0)
    return;
  
  GridElement &elem = get(ID.x(), ID.y());
  
  if( (!elem.obstacle) && obstacle){
      elem.init_confidences(min_depth, max_depth, depth_resolution);
    
      elem.obstacle = true;
      elem.obstacle_confidence = confidence + 0.5 - (confidence * 0.5);
      setObstacleDepthConfidence(elem.obstacle_depth_confidence, min_depth, max_depth, confidence, obstacle);
      elem.obstacle_count++;
    
  }  
  else if(obstacle && confidence > 0.0){
   
    setObstacleDepthConfidence(elem.obstacle_depth_confidence, min_depth, max_depth, confidence, obstacle);
    
    if(obstacle == false && elem.obstacle && confidence > 0){ //we do not see an old feature --> reduce feature confidence
      //feature.obstacle_confidence = ((1.0 - confidence) + f.obstacle_confidence) * 0.5;
      
      //Confidence, that the cell is empty
      double confidence_empty = (1.0 - elem.obstacle_confidence) + confidence - ((1.0 - elem.obstacle_confidence) * confidence);
      
      //Obstacle ceonfidence is reverse of empty-confidence
      elem.obstacle_confidence = 1.0 - confidence_empty;
      
    }else if(obstacle == elem.obstacle && obstacle && confidence > 0){ // We recognized the feature before, update the confidence 
      //feature.obstacle_confidence = (confidence + f.obstacle_confidence) * 0.5 ;
      elem.obstacle_confidence = elem.obstacle_confidence + confidence - (elem.obstacle_confidence * confidence);
      elem.obstacle_count++;
    }
    else if(obstacle && elem.obstacle == false && confidence > 0){ //we recognize this feature for the first time
      elem.obstacle = true;
      
       //The confidence before was 0.5, because we did not know, if the cell is empty or occupied
      elem.obstacle_confidence = confidence + 0.5 - (confidence * 0.5);
      elem.obstacle_count++;
    }
      
    if(elem.obstacle_confidence <= 0.0 && !elem.is_obstacle(0.0000001)){ // We have no confidence in this feature
      elem.obstacle = false; 
    }    
    
    
    
    
  }
   
}


void DepthObstacleGrid::setObstacleDepthConfidence(std::vector<ObstacleConfidence> &depth_vector, double min, double max, double confidence, bool obstacle){
  
  for(std::vector<ObstacleConfidence>::iterator it = depth_vector.begin(); it != depth_vector.end(); it++){
    
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




base::samples::Pointcloud DepthObstacleGrid::getCloud(){
  
  base::samples::Pointcloud result;
   
  for(std::vector<GridElement>::iterator it = grid.begin(); it != grid.end(); it++){
      
      if(it->depth_variance < INFINITY){
    
        base::Vector3d vec(it->pos.x(), it->pos.y(), it->depth);
        result.points.push_back(vec);
        result.colors.push_back(base::Vector4d(it->depth_variance, it->depth_variance, it->depth_variance, 1.0  ) );
      }
      
      if(it->obstacle_confidence > 0.0 && it->obstacle){
        
        base::Vector3d vec(it->pos.x(), it->pos.y(), it->obstacle_confidence);
        result.points.push_back(vec);
        result.colors.push_back(base::Vector4d(1.0, 0.0, 0.0, 1.0 ) );        
      }
    
  }
  
  return result;
}


void DepthObstacleGrid::getSimpleGrid( uw_localization::SimpleGrid &simple_grid ,double confidence_threshold, int count_threshold ){
 std::cout << "Get simple grid " << std::endl; 
 simple_grid.init(position, span, resolution); 
  
 for(std::vector<GridElement>::iterator it = grid.begin(); it != grid.end(); it++){
   
   if( !std::isnan(it->depth) ){
     
          SimpleGridElement elem;
          
          simple_grid.getCell(it->pos.x(), it->pos.y(), elem);
          
          elem.depth = it->depth;
          
          simple_grid.setCell(it->pos.x(), it->pos.y(), elem);    
   }
   
   if( it->obstacle || it->obstacle_count > count_threshold){
     
          SimpleGridElement elem;
          
          simple_grid.getCell(it->pos.x(), it->pos.y(), elem);
          
          elem.obstacle = true;
          elem.obstacle_conf = it->obstacle_confidence;
          
          if(it->obstacle_count > count_threshold){
            elem.obstacle_conf = 1.0;
          }
          
          simple_grid.setCell(it->pos.x(), it->pos.y(), elem);  
          std::cout << "Add cell " << it->pos.transpose() << std::endl;
   } 
   
   
 }
  std::cout << "----------------" << std::endl;
  
}



void DepthObstacleGrid::reduce_weights(double val){
  
  for(std::vector<GridElement>::iterator it = grid.begin(); it != grid.end(); it++){
    it->obstacle_weight -= val;
    
    if(it->obstacle_weight <= 0.0)
      it->obstacle = false;
    
  }
}

  

