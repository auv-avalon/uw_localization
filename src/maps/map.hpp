#ifndef UW_LOCALIZATION_MAPS_HPP
#define UW_LOCALIZATION_MAPS_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace uw_localization {

class Map {
public:
  Map(const Eigen::Vector3d& limitations, const Eigen::Translation3d& translation);
  ~Map();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  virtual bool belongsToWorld(const Eigen::Vector3d& point) const;

  virtual void setWorldLimitations(double width, double height, double depth);
  virtual Eigen::Vector3d getLimitations() const;

protected:
  Eigen::Vector3d limitations;
  Eigen::Translation3d translation;
};




}

#endif
