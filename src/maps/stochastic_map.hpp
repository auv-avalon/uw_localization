#ifndef UW_LOCALIZATION_MAPS_STOCHASTIC_MAPS
#define UW_LOCALIZATION_MAPS_STOCHASTIC_MAPS

#include <string>
#include <Eigen/Core>
#include <machine_learning/GaussianParameters.hpp>
#include "map.hpp"

namespace uw_localization {

const int NODE_GROUP = 0;
const int NODE_LANDMARK = 0;

class Node {
public:
  Node(const std::string& caption = "");
  virtual ~Node();

  virtual int getNodeType() const { return NODE_GROUP; }
  
  void addChild(Node* child);
  Node* getChild(unsigned index);
  void removeChild(unsigned index);
  void removeChild(Node* child);

  unsigned getChildSize() const;

  const std::string& getCaption() const { return caption; }
  std::string& getCaption() { return caption; }

private:
  std::vector<Node*> children;
  std::string caption;
};


class LandmarkNode : public Node {
public:
  LandmarkNode(const std::string& caption, const Eigen::Vector3d& mean, const Eigen::Matrix3d& cov);
  virtual ~LandmarkNode();

  virtual int getNodeType() const { return NODE_LANDMARK; }

  machine_learning::GaussParam<3> params;
};


class StochasticMap : public Map {
public:
  StochasticMap(double w, double h, double d);
  virtual ~StochasticMap();

private:
  Node* root;
};

}

#endif
