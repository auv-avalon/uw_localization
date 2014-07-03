#include "grid_map.hpp"

using namespace uw_localization;


GridMap::GridMap(base::Vector2d position, base::Vector2d span, double resolution  ){
  
  std::cout << "Set up grid map:" << std::endl;
  std::cout << "Center: " << position.transpose() << std::endl;
  std::cout << "Span: " << span.transpose() << std::endl;
  std::cout << "Resolution: " << resolution << std::endl;
  
  this->position = position;
  this->span = span;
  this->resolution = resolution;
  
  grid.resize( (span.x()/resolution) * (span.y()/resolution));
  
  for(double x = 0.0; x < span.x(); x = x + resolution){

    for( double y = 0.0; y < span.y(); y = y + resolution){

      setDepth(x - position.x() , y - position.y() , 0.0, 0.0);
    }
    
  }
  std::cout << "Map size: " << grid.size() << std::endl;
  
}


GridMap::~GridMap(){
  
  
}

double GridMap::getDepth( double x, double y){
  
  int idX = ((x + position.x()) / span.x()) * (span.x()/resolution);
  int idY = ((y + position.y()) / span.y()) * (span.y()/resolution);
  
  GridElement elem = get(idX, idY);
  
  if(elem.depth_confidence <= 0.0)
    return NAN;
  else
    return elem.depth;
  
}

void GridMap::setDepth(double x, double y, double depth, double confidence){
  
  int idX = ((x + position.x()) / span.x()) * (span.x()/resolution);
  int idY = ((y + position.y()) / span.y()) * (span.y()/resolution);
  //std::cout << "idX: " << idX << " span: " << span.x() << " resolution: " << resolution << " x: " << x << " pos: " << position.x() << std::endl;
  
  GridElement elem = get(idX, idY);
  
  if(confidence > elem.depth_confidence){
    
    if(elem.depth_confidence == -1.0) //Only set initial position
      elem.pos = base::Vector2d(x,y);

    elem.depth_confidence = confidence;
    elem.depth = depth;
    
    set(idX, idY, elem);
  }  
  
}

bool GridMap::getObstacle( double x, double y){
  
  int idX = ((x + position.x()) / span.x()) * (span.x()/resolution);
  int idY = ((y + position.y()) / span.y()) * (span.y()/resolution);
  
  GridElement elem = get(idX, idY);
  
  if(elem.obstacle_confidence <= 0.0)
    return false;
  else
    return elem.obstacle;
  
}



void GridMap::setObstacle(double x, double y, bool obstacle, double confidence){

  int idX = ((x + position.x()) / span.x()) * (span.x()/resolution);
  int idY = ((y + position.y()) / span.y()) * (span.y()/resolution);
  //std::cout << "idX: " << idX << " span: " << span.x() << " resolution: " << resolution << " x: " << x << " pos: " << position.x() << std::endl;
  
  GridElement elem = get(idX, idY);
  
  if(confidence > elem.obstacle_confidence){
    
    elem.obstacle_confidence = confidence;
    elem.obstacle = obstacle;
    
    set(idX, idY, elem);
  }   
}


base::samples::Pointcloud GridMap::getCloud(){
  
  base::samples::Pointcloud result;
   
  for(std::vector<GridElement>::iterator it = grid.begin(); it != grid.end(); it++){
      
      if(it->depth_confidence > 0.0){
    
        base::Vector3d vec(it->pos.x(), it->pos.y(), it->depth);
        result.points.push_back(vec);
        result.colors.push_back(base::Vector4d(it->depth_confidence, it->depth_confidence, it->depth_confidence, 1.0  ) );
      }
      
      if(it->obstacle_confidence > 0.0 && it->obstacle){
        
        base::Vector3d vec(it->pos.x(), it->pos.y(), 0.1);
        result.points.push_back(vec);
        result.colors.push_back(base::Vector4d(1.0, 0.0, 0.0, 1.0 ) );        
      }
    
  }
  
  return result;
}

 
void GridMap::set(int x, int y, GridElement val){
  int id = (y * (span.x()/resolution) ) + x;
  if( id < grid.size() && id >= 0)
    grid[ (y * (span.x()/resolution) ) + x ] = val;
  else{
    std::cout << "Invalid get:   x: " << x << " y: " << y << " id: " << id << " girdsize: " << grid.size() << std::endl;
  }
  
}

GridElement GridMap::get(int x, int y){
  int id = (y * (span.x()/resolution) ) + x;
  
  if( id  < grid.size() && id >= 0)
    return grid[ (y * (span.x()/resolution) ) + x];
  else{
    std::cout << "Invalid set:  x: " << x << " y: " << y << " id: " << id << " gridsize: " << grid.size() << std::endl;
    return GridElement();
  }
}