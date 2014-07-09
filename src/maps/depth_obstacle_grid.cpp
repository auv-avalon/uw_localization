#include "depth_obstacle_grid.hpp"

using namespace uw_localization;


DepthObstacleGrid::~DepthObstacleGrid(){}


double DepthObstacleGrid::getDepth( double x, double y){
  
  std::pair<int, int > ID = getCellID(x,y);
  
  GridElement elem = get(ID.first, ID.second);
  
  if(elem.depth_confidence <= 0.0)
    return NAN;
  else
    return elem.depth;
  
}

void DepthObstacleGrid::setDepth(double x, double y, double depth, double confidence){
  
  std::pair<int, int > ID = getCellID(x,y);
  //std::cout << "idX: " << idX << " span: " << span.x() << " resolution: " << resolution << " x: " << x << " pos: " << position.x() << std::endl;
  
  GridElement elem = get(ID.first, ID.second);
   //std::cout << "Set depth: " << depth << " conf: " << confidence << " elemconf: " << elem.depth_confidence << " x: " << x << " y: " << y << std::endl;
  if(confidence > elem.depth_confidence){
    
    if(elem.depth_confidence == -1.0) //Only set initial position
      elem.pos = base::Vector2d(x,y);

    elem.depth_confidence = confidence;
    elem.depth = depth;
    
    //std::cout << "Set depth: " << elem.depth << " conf: " << elem.depth_confidence << " x: " << idX << " y: " << idY << std::endl;
    set(ID.first, ID.second, elem);
  }  
  
}

bool DepthObstacleGrid::getObstacle( double x, double y){
  
  std::pair<int, int > ID = getCellID(x,y);
  
  GridElement elem = get(ID.first, ID.second);
  
  if(elem.obstacle_confidence <= 0.0)
    return false;
  else
    return elem.obstacle;
  
}

void DepthObstacleGrid::setObstacle(double x, double y, bool obstacle, double confidence){

  std::pair<int, int > ID = getCellID(x,y);
  //std::cout << "idX: " << idX << " span: " << span.x() << " resolution: " << resolution << " x: " << x << " pos: " << position.x() << std::endl;
  
  GridElement elem = get(ID.first, ID.second);
  
  if(!obstacle && elem.obstacle){
    elem.obstacle_weight -= confidence;
    
    if(elem.obstacle_weight <= 0.0)
      elem.obstacle = false;
    
    elem.obstacle_confidence = confidence;
    set(ID.first, ID.second, elem);
    
  }
  
  if(obstacle){
    
    if(elem.obstacle && elem.obstacle_weight > 0.0)
      elem.obstacle_weight += confidence;
    else
      elem.obstacle_weight = confidence;
    
    elem.obstacle = true;
    elem.obstacle_confidence = confidence;
    std::cout << "Set obstacle true: " << x << " " << y << " weight: " << elem.obstacle_weight << std::endl;
    set(ID.first, ID.second, elem);    
  }
   
}



base::samples::Pointcloud DepthObstacleGrid::getCloud(){
  
  base::samples::Pointcloud result;
   
  for(std::vector<GridElement>::iterator it = grid.begin(); it != grid.end(); it++){
      
      if(it->depth_confidence > 0.0){
    
        base::Vector3d vec(it->pos.x(), it->pos.y(), it->depth);
        result.points.push_back(vec);
        result.colors.push_back(base::Vector4d(it->depth_confidence, it->depth_confidence, it->depth_confidence, 1.0  ) );
      }
      
      if(it->obstacle_confidence > 0.0 && it->obstacle){
        
        base::Vector3d vec(it->pos.x(), it->pos.y(), it->obstacle_weight);
        result.points.push_back(vec);
        result.colors.push_back(base::Vector4d(1.0, 0.0, 0.0, 1.0 ) );        
      }
    
  }
  
  return result;
}



void DepthObstacleGrid::reduce_weights(double val){
  
  for(std::vector<GridElement>::iterator it = grid.begin(); it != grid.end(); it++){
    it->obstacle_weight -= val;
    
    if(it->obstacle_weight <= 0.0)
      it->obstacle = false;
    
  }
}

  

