#include "grid_map.hpp"

using namespace uw_localization;


GridMap::GridMap(base::Vector2d position, base::Vector2d span, double resolution  ){
  
  this->position = position;
  this->span = span;
  this->resolution = resolution;
  
  grid.resize( (span.x()/resolution) * (span.y()/resolution));
  
  
}


GridMap::~GridMap(){
  
  
}

double GridMap::getDepth( double x, double y){
  
  int idX = ((x + position.x()) / span.x()) * (span.x()/resolution);
  int idY = ((y + position.y()) / span.y()) * (span.y()/resolution);
  
  GridElement elem = get(idX, idY);
  
  if(elem.confidence <= 0.0)
    return NAN;
  else
    return elem.depth;
  
  return 0.0;
}

void GridMap::setDepth(double x, double y, double depth, double confidence){
  
  int idX = ((x + position.x()) / span.x()) * (span.x()/resolution);
  int idY = ((y + position.y()) / span.y()) * (span.y()/resolution);
  //std::cout << "idX: " << idX << " span: " << span.x() << " resolution: " << resolution << " x: " << x << " pos: " << position.x() << std::endl;
  
  GridElement elem = get(idX, idY);
  
  if(confidence > elem.confidence){
    elem.confidence = confidence;
    elem.depth = depth;
    elem.pos = base::Vector2d(x,y);
    set(idX, idY, elem);
  }  
  
}


std::vector<base::Vector3d> GridMap::getCloud(){
  
  std::vector<base::Vector3d> result;
  result.clear();
  
  for(std::vector<GridElement>::iterator it = grid.begin(); it != grid.end(); it++){
      
      if(it->confidence > 0.0){
    
        base::Vector3d vec(it->pos.x(), it->pos.y(), it->depth);
        result.push_back(vec);     
      }
    
  }
  
  return result;
}

 
void GridMap::set(int x, int y, GridElement val){
  int id = (y * (span.x()/resolution) ) + x;
  if( id < grid.size() && id >= 0)
    grid[ (y * (span.x()/resolution) ) + x ] = val;
  else{
    //std::cout << "x: " << x << " y: " << y << " id: " << id << " girdsize: " << grid.size() << std::endl;
  }
  
}

GridElement GridMap::get(int x, int y){
  int id = (y * (span.x()/resolution) ) + x;
  
  if( id  < grid.size() && id >= 0)
    return grid[ (y * (span.x()/resolution) ) + x];
  else
    return GridElement();
}