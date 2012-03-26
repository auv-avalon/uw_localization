#include "stochastic_map.hpp"

namespace uw_localization {

Node::Node(const std::string& caption)
{
}


Node::~Node()
{
}


void Node::addChild(Node* child)
{
}


Node* Node::getChild(unsigned index)
{
    return 0;
}


void Node::removeChild(unsigned index)
{
}


void Node::removeChild(Node* child)
{
}


unsigned Node::getChildSize() const
{
    return 0;
}



// ----------------------------------------------------------------------------


LandmarkNode::LandmarkNode(const std::string& caption, const Eigen::Vector3d& mean, const Eigen::Matrix3d& cov)
{
}


LandmarkNode::~LandmarkNode()
{
}


// ----------------------------------------------------------------------------


StochasticMap::StochasticMap(double w, double h, double d)
    : Map(w, h, d)
{
    root = new Node("root");
}


StochasticMap::~StochasticMap()
{
    delete root;
}



}
