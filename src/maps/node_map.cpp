#include "node_map.hpp"
#include <stack>
#include <limits>
#include <fstream>
#include <algorithm>
#include <boost/assert.hpp>

using namespace machine_learning;

namespace uw_localization {

Node::Node(const std::string& caption) : caption(caption)
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


boost::tuple<Node*, double> Node::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x)
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

    boost::tuple<Node*, double> min(0, std::numeric_limits<double>::max());

    if(getCaption() == current || current.empty()) {
        for(unsigned i = 0; i < children.size(); i++) {
            boost::tuple<Node*, double> tmp = children[i]->getNearestDistance(next, v, x);

            if(tmp.get<1>() < min.get<1>())
                min = tmp;
        }
    }

    return min;
}


std::vector<Node*> Node::getLeafs(const std::string& caption) 
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

    std::vector<Node*> nodes;

    if(getCaption() == current || current.empty()) {
        for(unsigned i = 0; i < children.size(); i++) {
            std::vector<Node*> v;
            switch(children[i]->getNodeType()) {
                case NODE_GROUP:
                    v = children[i]->getLeafs(next);
                    nodes.insert(nodes.end(), v.begin(), v.end());
                    break;

                case NODE_LINE:
                case NODE_LANDMARK:
                    nodes.push_back(children[i]);
                    break;
                default:
                    break;
            }
        }
    }

    return nodes;
}



// ----------------------------------------------------------------------------


LandmarkNode::LandmarkNode(const Eigen::Vector3d& mean, const Eigen::Matrix3d& cov, const std::string& caption)
    : Node(caption), params(mean, cov), drawer(machine_learning::Random::multi_gaussian<3>(mean, cov))
{}


LandmarkNode::~LandmarkNode()
{}


Eigen::Vector3d LandmarkNode::draw()
{
    return drawer();
}


boost::tuple<Node*, double> LandmarkNode::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x)
{
    return boost::tuple<Node*, double>(this, params.mahalanobis(v));
}

// ----------------------------------------------------------------------------


LineNode::LineNode(const Line& line, double height, const std::string& caption)
    : Node(caption), line(line), height(height)
{}


LineNode::~LineNode()
{}


Eigen::Vector3d LineNode::draw()
{
    return Eigen::Vector3d();
}


boost::tuple<Node*, double> LineNode::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x)
{
    Line measurement = Line::fromTwoPoints(x, v);
    Point line_point;

    double line_lambda = line.lambda(measurement);
    if(line_lambda == INFINITY)
	line_lambda = line.lambda(v);

    if(line_lambda < 0.0)
	line_point = line.from();
    else if(line_lambda > 1.0)
	line_point = line.to();
    else
	line_point = line.point(line_lambda);
        
    return boost::tuple<Node*, double>(this, (v - line_point).norm());
}



// ----------------------------------------------------------------------------

NodeMap::NodeMap(const std::string& map) : Map(), root(0) 
{
    std::ifstream fin(map.c_str());
    fromYaml(fin);
}


NodeMap::NodeMap(const Eigen::Vector3d& limits, const Eigen::Translation3d& t, Node* root)
    : Map(limits, t), root(root)
{
}


NodeMap::~NodeMap()
{
    delete root;
}


std::vector<boost::tuple<Node*, Eigen::Vector3d> > NodeMap::drawSamples(const std::string& caption, int numbers) 
{
    std::vector<boost::tuple<Node*, Eigen::Vector3d> > list;
    std::vector<Node*> nodes = root->getLeafs(caption);
    UniformIntRandom rand = Random::uniform_int(0, nodes.size() - 1);

    for(unsigned i = 0; i < numbers; i++) {
        Node* node = nodes[rand()];
        list.push_back(boost::tuple<Node*, Eigen::Vector3d>(node, node->draw()));
    }

    return list;
}


boost::tuple<Node*, double> NodeMap::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x)
{
    return root->getNearestDistance(caption, v, x);
}


bool NodeMap::toYaml(std::ostream& stream)
{
    return false;
}


void operator>>(const YAML::Node& node, Eigen::Vector3d& v)
{
    node[0] >> v.x();
    node[1] >> v.y();
    node[2] >> v.z();
}

void operator>>(const YAML::Node& node, Eigen::Vector2d& v)
{
    node[0] >> v.x();
    node[1] >> v.y();
}



void operator>>(const YAML::Node& node, Eigen::Matrix3d& cov)
{
    unsigned index = 0;
    for(unsigned i = 0; i < 3; i++) {
        for(unsigned j = 0; j < 3; j++) {
            node[index++] >> cov(i, j);
        }
    }
}


bool NodeMap::fromYaml(std::istream& stream)
{
    if(stream.fail()) {
        std::cerr << "Could not open input stream" << std::endl;
        return false;
    }

    if(root)
        delete root;

    root = new Node("root");

    YAML::Parser parser(stream);
    YAML::Node doc;
    Eigen::Vector3d parse_limit;
    Eigen::Vector3d parse_translation;

    while(parser.GetNextDocument(doc)) {
        doc["metrics"] >> parse_limit;
        doc["reference"] >> parse_translation;
	const YAML::Node& root_node = doc["root"];

	parseYamlNode(root_node, root);
    }

    limitations = parse_limit;
    translation = Eigen::Translation3d(parse_translation);

    return true;
}



void NodeMap::parseYamlNode(const YAML::Node& node, Node* root)
{
        std::string caption = "";
	if(node.GetType() == YAML::CT_MAP && node.FindValue("mean")) {
		Eigen::Vector3d mean;
		Eigen::Matrix3d cov;

		node["mean"] >> mean;
		node["cov"] >> cov;

		if(node.FindValue("caption"))
			node["caption"] >> caption;

		root->addChild(new LandmarkNode(mean, cov, caption));
        } else if(node.GetType() == YAML::CT_MAP && node.FindValue("line_from")) {
		Eigen::Vector3d from;
		Eigen::Vector3d to;
		double height;

		node["line_from"] >> from;
		node["line_to"] >> to;
		node["height"] >> height;

		if(node.FindValue("caption"))
			node["caption"] >> caption;

		Line line = Line::fromTwoPoints(from, to);
		
		root->addChild(new LineNode(line, height, caption));

	} else if(node.GetType() == YAML::CT_MAP) {
		for(YAML::Iterator it = node.begin(); it != node.end(); ++it) {
			std::string groupname;
			it.first() >> groupname;

			Node* group = new Node(groupname);
			
                        root->addChild(group);
			parseYamlNode(it.second(), group);
		}
	} else if(node.GetType() == YAML::CT_SEQUENCE) {
		for(YAML::Iterator it = node.begin(); it != node.end(); ++it) {
			parseYamlNode(*it, root);
		}
	} else {
		std::cerr << "Failure in map yaml parsing." << std::endl;
		BOOST_ASSERT(0);
	}
}



MixedMap NodeMap::getMap()
{
    MixedMap map;

    map.limitations = getLimitations();
    map.translation = Eigen::Vector3d(translation.x(), translation.y(), translation.z());

    std::vector<Node*> leafs = root->getLeafs();
    Landmark landmark;
    Linemark linemark;

    for(unsigned i = 0; i < leafs.size(); i++) {
        switch(leafs[i]->getNodeType()) {
        case NODE_LANDMARK:
    	    landmark.caption = leafs[i]->getCaption();
	    landmark.mean = dynamic_cast<LandmarkNode*>(leafs[i])->mean();
	    landmark.covariance = dynamic_cast<LandmarkNode*>(leafs[i])->covariance();
        
            map.landmarks.push_back(landmark);

        case NODE_LINE:
	    linemark.from = dynamic_cast<LineNode*>(leafs[i])->getLine().from();
	    linemark.to   = dynamic_cast<LineNode*>(leafs[i])->getLine().to();
	    linemark.height = dynamic_cast<LineNode*>(leafs[i])->getHeight();

	    map.lines.push_back(linemark);
	     	    
        default:
	    break;
        }
    }

    return map;
}


}
