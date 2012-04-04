#include "map.hpp"

namespace uw_localization {


Map::Map() 
 : limitations(Eigen::Vector3d(10.0, 10.0, 10.0))
 , translation(Eigen::Translation3d(0.0, 0.0, 0.0))
{
}



Map::Map(const Eigen::Vector3d& l, const Eigen::Translation3d& t)
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


bool Map::belongsToWorld(const Eigen::Vector3d& p) const
{
    Eigen::Vector3d null(0.0, 0.0, 0.0);
    
    Eigen::Vector3d ref = Eigen::Affine3d(translation) * null;
    Eigen::Vector3d l = ref + limitations;

    return (p.x() >= ref.x() && p.x() <= l.x()) 
        && (p.y() >= ref.y() && p.y() <= l.y())
        && (p.z() >= ref.z() && p.z() <= l.z());
}

}
