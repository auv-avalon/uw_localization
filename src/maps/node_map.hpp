#ifndef UW_LOCALIZATION_MAPS_STOCHASTIC_MAPS
#define UW_LOCALIZATION_MAPS_STOCHASTIC_MAPS

#include <string>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <machine_learning/GaussianParameters.hpp>
#include <machine_learning/RandomNumbers.hpp>
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include "map.hpp"
#include "../math/geometry.hpp"
#include "../types/map.hpp"
#include "../types/environment.hpp"

namespace uw_localization {

const int NODE_GROUP = 0;
const int NODE_LANDMARK = 1;
const int NODE_LINE = 2;
const int NODE_BOX = 3;

class LandmarkNode;

class Node {
public:
  Node(const std::string& caption = "");
  virtual ~Node();
  
  void addChild(Node* child);
  Node* getChild(unsigned index);
  void removeChild(unsigned index);
  void removeChild(Node* child);

  unsigned getChildSize() const;

  virtual int getNodeType() const { return NODE_GROUP; }
  
  virtual boost::tuple<Node*, double, Eigen::Vector3d> getNearestDistance(const std::string& caption, 
	const Eigen::Vector3d& v, const Eigen::Vector3d& x);

  virtual Eigen::Vector3d draw() {
     throw std::runtime_error("not supported by Node class");
  }

  std::vector<Node*> getLeafs(const std::string& caption = "");

  const std::string& getCaption() const { return caption; }
  std::string& getCaption() { return caption; }

private:
  std::vector<Node*> children;
  std::string caption;
};


class LineNode : public Node {
public:
  LineNode(const Line& line, double height, const std::string& caption = "");
  virtual ~LineNode();

  virtual int getNodeType() const { return NODE_LINE; }
  
  /**
   * @param caption: label of the nodes
   * @param v: meassurement
   * @param x: start point of meassurement (vehicle postion)
   */
  virtual boost::tuple<Node*, double, Eigen::Vector3d> getNearestDistance(const std::string& caption, 
	const Eigen::Vector3d& v, const Eigen::Vector3d& x);

  virtual Eigen::Vector3d draw();
  
  const Line& getLine() const { return line; }
  const double getHeight() const { return height; }

private:
  Line line; 
  double height;
};


class LandmarkNode : public Node {
public:
  LandmarkNode(const Eigen::Vector3d& point, const std::string& caption = "");
  virtual ~LandmarkNode();

  virtual int getNodeType() const { return NODE_LANDMARK; }
  
  virtual boost::tuple<Node*, double, Eigen::Vector3d> getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x);

  virtual Eigen::Vector3d draw();

  const Eigen::Vector3d& point() const { return _point; }

private:
  Eigen::Vector3d _point;
};

class BoxNode : public Node {
public:  
  BoxNode(const Eigen::Vector3d position, const Eigen::Vector3d span, const std::string& caption = "");
  virtual ~BoxNode();
  
  virtual int getNodeType() const {return NODE_BOX; }
  
  /**
   * @param caption: label of the nodes
   * @param v: direction of view (0, vertical opening angle, yaw)
   * @param x: position of view
   */
  virtual boost::tuple<Node*, double, Eigen::Vector3d> getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x);
  
  virtual Eigen::Vector3d draw();
  
  Eigen::Vector3d getPosition();
  Eigen::Vector3d getSpan();
  
private:
  Eigen::Vector3d position;
  Eigen::Vector3d span;
  double radius;
  
  Eigen::Vector3d right_upper_corner;
  Eigen::Vector3d right_under_corner;
  Eigen::Vector3d left_upper_corner;
  Eigen::Vector3d left_under_corner;
  
  Line upper_line;
  Line under_line;
  Line left_line;
  Line right_line;
    
};

class NodeMap : public Map {
public:
  NodeMap(const std::string& map);
  NodeMap(const Eigen::Vector3d& limits, const Eigen::Vector3d& t, Node* root);
  virtual ~NodeMap();

  virtual std::vector<boost::tuple<Node*, Eigen::Vector3d> > drawSamples(const std::string& caption, int numbers) const;

  virtual boost::tuple<Node*, double, Eigen::Vector3d> getNearestDistance(const std::string& caption, 
	const Eigen::Vector3d& v, const Eigen::Vector3d& x) const;
 
  virtual Environment getEnvironment();

  bool toYaml(std::ostream& stream);
  bool fromYaml(std::istream& stream);

private: 
  void parseYamlNode(const YAML::Node& node, Node* root);

private:
  Node* root;
};

void operator>>(const YAML::Node& node, Eigen::Vector3d& v);
void operator>>(const YAML::Node& node, Eigen::Vector2d& v);
void operator>>(const YAML::Node& node, Eigen::Matrix3d& cov);
}

#endif
