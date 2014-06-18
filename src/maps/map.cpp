#include "map.hpp"

namespace uw_localization {


Map::Map() 
 : limitations(Eigen::Vector3d(10.0, 10.0, 10.0))
 , translation(Eigen::Vector3d(0.0, 0.0, 0.0))
{
}



Map::Map(const Eigen::Vector3d& l, const Eigen::Vector3d& t)
 : limitations(l), translation(t)
{}


Map::~Map()
{
}


void Map::setWorldLimitations(double w, double h, double d)
{
    limitations = Eigen::Vector3d(w, h, d);
}

Eigen::Vector3d Map::getLimitations() const
{
    return limitations;
}

Eigen::Vector3d Map::getTranslation() const
{
  return translation;
}


bool Map::belongsToWorld(const Eigen::Vector3d& p) const
{
    Eigen::Vector3d ref = translation;
    Eigen::Vector3d l = translation + limitations;

    return (p.x() >= ref.x() && p.x() <= l.x()) 
        && (p.y() >= ref.y() && p.y() <= l.y())
        && (p.z() >= ref.z() && p.z() <= l.z());
}

}
