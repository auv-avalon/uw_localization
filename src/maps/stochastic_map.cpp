#include "stochastic_map.hpp"
#include <limits>
#include <algorithm>

using namespace machine_learning;

namespace uw_localization {

Node::Node(const std::string& caption)
{
}


Node::~Node()
{
    for(unsigned i = 0; i < children.size(); i++)
        delete children[i];
}


void Node::addChild(Node* child)
{
    children.push_back(child);
}


Node* Node::getChild(unsigned index)
{
    return children[index];
}


void Node::removeChild(unsigned index)
{
    children.erase(children.begin() + index);
}


void Node::removeChild(Node* child)
{
    std::vector<Node*>::iterator it;

    for(it = children.begin(); it != children.end(); it++) {
        if((*it) == child) {
            children.erase(it);
            return;
        }
    }
}


unsigned Node::getChildSize() const
{
    return children.size();
}


boost::tuple<LandmarkNode*, double> Node::getProbability(const std::string& caption, const Eigen::Vector3d& v)
{
    size_t found = caption.find(".");
    std::string current;
    std::string next;

    if(found != std::string::npos) {
        current = caption.substr(0, found); 
        next = caption.substr(found + 1);
    } else {
        current = caption;
    }

    boost::tuple<LandmarkNode*, double> max(0, std::numeric_limits<double>::min());

    if(getCaption() == current || current.empty()) {
        for(unsigned i = 0; i < children.size(); i++) {
            boost::tuple<LandmarkNode*, double> tmp = children[i]->getProbability(next, v);

            if(tmp.get<1>() > max.get<1>())
                max = tmp;
        }
    }

    return max;
}


std::vector<LandmarkNode*> Node::getLandmarks(const std::string& caption) 
{
    size_t found = caption.find(".");
    std::string current;
    std::string next;

    if(found != std::string::npos) {
        current = caption.substr(0, found); 
        next = caption.substr(found + 1);
    } else {
        current = caption;
    }

    std::vector<LandmarkNode*> nodes;

    if(getCaption() == current || current.empty()) {
        for(unsigned i = 0; i < children.size(); i++) {
            std::vector<LandmarkNode*> v;
            switch(children[i]->getNodeType()) {
                case NODE_GROUP:
                    v = children[i]->getLandmarks(next);
                    nodes.insert(nodes.end(), v.begin(), v.end());
                    break;

                case NODE_LANDMARK:
                    nodes.push_back(dynamic_cast<LandmarkNode*>(children[i]));
                    break;
                default:
                    break;
            }
        }
    }

    return nodes;
}



// ----------------------------------------------------------------------------


LandmarkNode::LandmarkNode(const std::string& caption, const Eigen::Vector3d& mean, const Eigen::Matrix3d& cov)
    : Node(caption), params(mean, cov), drawer(machine_learning::Random::multi_gaussian<3>(mean, cov))
{
}


LandmarkNode::~LandmarkNode()
{
}


Eigen::Vector3d LandmarkNode::draw()
{
    return drawer();
}


boost::tuple<LandmarkNode*, double> LandmarkNode::getProbability(const std::string& caption, const Eigen::Vector3d& v)
{
    return boost::tuple<LandmarkNode*, double>(this, params.gaussian(v));
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


std::vector<boost::tuple<LandmarkNode*, Eigen::Vector3d> > StochasticMap::drawSamples(const std::string& caption, int numbers) 
{
    std::vector<boost::tuple<LandmarkNode*, Eigen::Vector3d> > list;
    std::vector<LandmarkNode*> nodes = root->getLandmarks(caption);
    UniformIntRandom rand = Random::uniform_int(0, nodes.size() - 1);

    for(unsigned i = 0; i < numbers; i++) {
        LandmarkNode* node = nodes[rand()];
        list.push_back(boost::tuple<LandmarkNode*, Eigen::Vector3d>(node, node->draw()));
    }

    return list;
}


boost::tuple<LandmarkNode*, double> StochasticMap::getProbability(const std::string& caption, const Eigen::Vector3d& v)
{
    return root->getProbability(caption, v);
}


LandmarkMap StochasticMap::getMap()
{
    LandmarkMap map;

    map.limitations = getLimitations();

    std::vector<LandmarkNode*> landmarks = root->getLandmarks();

    for(unsigned i = 0; i < landmarks.size(); i++) {
        Landmark mark;

        mark.caption = landmarks[i]->getCaption();
        mark.mean = landmarks[i]->mean();
        mark.covariance = landmarks[i]->covariance();
    }
}


}
