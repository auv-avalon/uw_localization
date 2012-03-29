#ifndef UW_LOCALIZATION_MAPS_HPP
#define UW_LOCALIZATION_MAPS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uw_localization {

class Map {
  static const double UNLIMITED = -1;

public:
  Map(double width = UNLIMITED, double height = UNLIMITED, double depth = UNLIMITED);
  ~Map();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual bool belongsToWorld(const Eigen::Vector3d& point) const;

  virtual void setWorldLimitations(double width, double height, double depth);
  virtual Eigen::Vector3d getLimitations() const;

private:
  double width;
  double height;
  double depth;
};




}

#endif
