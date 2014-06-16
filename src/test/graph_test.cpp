#include "../maps/node_map.hpp"
#include <Eigen/Core>


using namespace uw_localization;


int main(int argc, char** argv)
{
    //std::cout << "Hallo Test" << std::endl;
    
    
    NodeMap* map;
    map = new NodeMap();
    map->fromYaml("/home/fabio/avalon/avalon/uw_localization/src/test/map.yml");
    
    Eigen::Vector3d pos, direction, direction_wall;
    pos << -7.0, 0.0, 0.0;
    direction << 0.0, 0.1, 0.0;
    direction_wall << 0.0, 0.0, 0.0;
    
    std::cout << "position: " << pos.transpose() << std::endl;
    
    for(double degree = 0.0; degree < M_PI*2.0; degree = degree + 0.1){
      direction[2] = degree;
      direction_wall[0] = cos(degree) * 10;
      direction_wall[1] = sin(degree) * 10;
      direction_wall[2] = 0.0;
      direction_wall += pos;
      
      std::cout << "YAW: " << degree << " Scan: " << direction_wall.transpose() << std::endl;
      
      double distance = map->getNearestDistance("root.box", direction , pos).get<1>();
      double distance_wall =  (map->getNearestDistance("root.wall",  direction_wall  , pos).get<2>() - pos).norm();
      std::cout << "Scanning yaw: " << degree << " Distance to box: " << distance << std::endl;
      std::cout << "Scanning yaw: " << degree << " Distance to wall: " << distance_wall << std::endl;
      std::cout << "-------" << std::endl;
    }  
    
    
}  
