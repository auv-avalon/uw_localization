#include "map.hpp"

namespace uw_localization {

Map::Map(double w, double h, double d)
 : width(w), height(h), depth(d) 
{}

Map::~Map()
{
}


void Map::setWorldLimitations(double w, double h, double d)
{
    width = w;
    height = h;
    depth = d;
}

Eigen::Vector3d Map::getLimitations() const
{
    return Eigen::Vector3d(width, height, depth);
}

bool Map::belongsToWorld(const Eigen::Vector3d& point) const
{
    Eigen::Vector3d p = point;//  * LimitsToWorld.linear();

    return ((p.x() >= 0.0 && p.x() <= width) || width < 0.0)
        && ((p.y() >= 0.0 && p.y() <= height) || height < 0.0)
        && ((p.z() >= 0.0 && p.z() <= depth) || depth < 0.0);
}

}
