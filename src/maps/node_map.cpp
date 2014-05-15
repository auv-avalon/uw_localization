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


boost::tuple<Node*, double, Eigen::Vector3d> Node::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x)
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

    boost::tuple<Node*, double, Eigen::Vector3d> min(0, std::numeric_limits<double>::max());

    if(getCaption() == current || current.empty()) {
        for(unsigned i = 0; i < children.size(); i++) {
            boost::tuple<Node*, double, Eigen::Vector3d> tmp = children[i]->getNearestDistance(next, v, x);

            if(tmp.get<1>() > 0 && tmp.get<1>() < min.get<1>())
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
                case NODE_BOX:
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


LandmarkNode::LandmarkNode(const Eigen::Vector3d& point, const std::string& caption)
    : Node(caption), _point(point)
{}


LandmarkNode::~LandmarkNode()
{}


Eigen::Vector3d LandmarkNode::draw()
{
    return _point;
}


boost::tuple<Node*, double, Eigen::Vector3d> LandmarkNode::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x)
{
    return boost::tuple<Node*, double, Eigen::Vector3d>(this, (v - _point).norm(), _point);
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

/*
boost::tuple<Node*, double, Eigen::Vector3d> LineNode::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x)
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

    double distance = (v - line_point).norm();

    double direction = measurement.lambda(line);

    if(direction <= 0.0 || direction == INFINITY)
        distance = -distance;

    return boost::tuple<Node*, double, Eigen::Vector3d>(this, distance, line_point);
}
*/

boost::tuple<Node*, double, Eigen::Vector3d> LineNode::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x){
  
  //Maximum line length for a line from x to wall, which intersects with wall
  double max_linelength = (line.from() - x).norm() + line.direction().norm();
  double yaw = std::atan2( v[1] - x[1], v[0] - x[0]);
  Line scan = Line::fromTwoPoints(x, x + Eigen::Vector3d(max_linelength * std::cos(yaw), max_linelength * std::sin(yaw),0.0) );
    
  Eigen::Vector3d intersection = line.intersectionPoint(scan);  
  
  //std::cout << "Wall: " << line.from().transpose() << " - " << line.to().transpose() << std::endl;
  //std::cout << "Wall scan: " << scan.from().transpose() << " - " <<  scan.to().transpose() << std::endl;
  //std::cout << "Intersection: " << intersection.transpose() << std::endl; 
  Eigen::Vector3d pos = x; //Measure posistion without depth
  pos[2] = 0.0;
  double distance = (pos - intersection).norm(); 
  
  if(isnan(distance)){
    return boost::tuple<Node*, double, Eigen::Vector3d>( this, INFINITY, intersection);
  }else{
    return boost::tuple<Node*, double, Eigen::Vector3d>( this, distance, intersection);
  }
} 


//-----------------------------------------------------------------------------

BoxNode::BoxNode(const Eigen::Vector3d position, const Eigen::Vector3d span, const std::string& caption)
    : Node(caption), position(position), span(span)
{
  radius = std::sqrt( std::pow( 0.5*span[0], 2.0) + std::pow( 0.5* span[1], 2.0));
  right_upper_corner = Eigen::Vector3d( position[0] + (0.5 * span[0]), position[1] + (0.5 * span[1]), 0.0);
  right_under_corner = Eigen::Vector3d( position[0] + (0.5 * span[0]), position[1] - (0.5 * span[1]) , 0.0);
  left_upper_corner = Eigen::Vector3d( position[0] - (0.5 * span[0]), position[1] + (0.5 * span[1]), 0.0);
  left_under_corner = Eigen::Vector3d( position[0] - (0.5 * span[0]), position[1] - (0.5 * span[1]) , 0.0);
  
  left_line = Line::fromTwoPoints(left_upper_corner, left_under_corner);
  right_line = Line::fromTwoPoints(right_upper_corner, right_under_corner);
  upper_line = Line::fromTwoPoints(right_upper_corner, left_upper_corner);
  under_line = Line::fromTwoPoints(right_under_corner, left_under_corner);

}

BoxNode::~BoxNode()
{}

boost::tuple<Node*, double, Eigen::Vector3d> BoxNode::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x){
  //return boost::tuple<Node*, double, Eigen::Vector3d>(this, 42, x);
  //Distance between object and particle, in the xy-plane
  double distanceX2Point = sqrt( pow(x[0] - position[0], 2.0) + pow(x[1] - position[1], 2.0));
  double verticalScanWidth = tan(v[1]) * distanceX2Point;  
  
  //Check, if the object is in scanning depth.
  if( position[2] + (0.5 * span[2]) < x[2] - verticalScanWidth || position[2] - (0.5 * span[2]) > x[2] + verticalScanWidth){
    std::cout << "Out of scanning depth" << std::endl;
    return boost::tuple<Node*, double, Eigen::Vector3d>(this, INFINITY, x);
  }
   
  //Create a line in the direction of the scan. Length is choosen as the distance between the particle and the objekt plus the radius of the object, to make sure, the line crosses the objekt
  Line scan = Line::fromPointDirection(x, Eigen::Vector3d( (distanceX2Point + radius) * cos(v[2]), (distanceX2Point + radius) * sin(v[2]), 0.0) );   
  
/*
    std::cout << "Box: " << position.transpose() << std::endl;
    std::cout << "Radius: " << radius << " lamda: " << scan.distance(position) << std::endl;
    std::cout << "Scan from " << scan.from().transpose() << " to " << scan.to().transpose() << std::endl; */
  
  //Check, if the object is out ouf the scan. If outside, ignore the box
  if( scan.distance(position) > radius){
    //std::cout << "Out of direction" << std::endl;
    return boost::tuple<Node*, double, Eigen::Vector3d>(this, INFINITY, x);
  }
    
  //if scan crosses the object, calculte intersections with object lines    
  double distance;

  Line width_line;
  Line height_line;
  
  if(x[0] < position[0])
    height_line = left_line;
  else
    height_line = right_line;  
  
  if(x[1] < position[1])
    width_line = under_line;
  else
    width_line = upper_line;
  
  Point intersection;  
  intersection = scan.intersectionPoint(width_line);
  //std::cout << "intersection " << intersection.transpose() << " with (" << width_line.from().transpose() << " , " << width_line.to().transpose() << " )" <<std::endl; 
  
  distance = std::sqrt( std::pow( x[0] - intersection[0], 2.0) + std::pow( x[1] - intersection[1], 2.0));
  
  Point temp_intersection = scan.intersectionPoint(height_line);
  
  double temp_distance = std::sqrt( std::pow( x[0] - temp_intersection[0], 2.0) + std::pow( x[1] - temp_intersection[1], 2.0));
  //std::cout << "intersection " << temp_intersection.transpose() << " with (" << height_line.from().transpose() << " , " << height_line.to().transpose() << " )" <<std::endl;
  
  if(temp_distance < distance || isnan(distance)){
    distance = temp_distance;
    intersection = temp_intersection;
  }
  
  if(isnan(distance))
    distance = INFINITY;
  
  std::cout << "Pos: " << x.transpose() << std::endl;
  std::cout << "Intersection: " << intersection.transpose() << std::endl;
  std::cout << "Distance: " << distance << std::endl;
  
  return boost::tuple<Node*, double, Eigen::Vector3d>(this, distance , intersection);
}

Eigen::Vector3d BoxNode::draw()
{
    return position;
}

Eigen::Vector3d BoxNode::getPosition()
{
  return position;
}

Eigen::Vector3d BoxNode::getSpan()
{
  return span;
}


// ----------------------------------------------------------------------------

NodeMap::NodeMap(const std::string& map) : Map(), root(0) 
{
    std::ifstream fin(map.c_str());
    fromYaml(fin);
}


NodeMap::NodeMap(const Eigen::Vector3d& limits, const Eigen::Vector3d& t, Node* root)
    : Map(limits, t), root(root)
{
}


NodeMap::~NodeMap()
{
    delete root;
}


std::vector<boost::tuple<Node*, Eigen::Vector3d> > NodeMap::drawSamples(const std::string& caption, int numbers) const 
{
    std::vector<boost::tuple<Node*, Eigen::Vector3d> > list;
    std::vector<Node*> nodes = root->getLeafs(caption);
    UniformIntRandom rand = Random::uniform_int(0, nodes.size() - 1);

    for(unsigned i = 0; i < static_cast<unsigned>(numbers); i++) {
        Node* node = nodes[rand()];
        list.push_back(boost::tuple<Node*, Eigen::Vector3d>(node, node->draw()));
    }

    return list;
}


boost::tuple<Node*, double, Eigen::Vector3d> NodeMap::getNearestDistance(const std::string& caption, const Eigen::Vector3d& v, const Eigen::Vector3d& x) const
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
    translation = parse_translation;

    return true;
}



void NodeMap::parseYamlNode(const YAML::Node& node, Node* root)
{
        std::string caption = "";
	if(node.Type() == YAML::NodeType::Map && node.FindValue("point")) {
		Eigen::Vector3d point;

		node["point"] >> point;

		if(node.FindValue("caption"))
			node["caption"] >> caption;

		root->addChild(new LandmarkNode(point, caption));
        } else if(node.Type() == YAML::NodeType::Map && node.FindValue("line_from")) {
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

	} else if(node.Type() == YAML::NodeType::Map && node.FindValue("position") && node.FindValue("span")) {
		Eigen::Vector3d pos;
		Eigen::Vector3d span;
		
		node["position"] >> pos;
		node["span"] >> span;
		
		if(node.FindValue("caption"))
		  node["caption"] >> caption;
		
		root->addChild(new BoxNode(pos, span, caption));	  
	  
	}else if(node.Type() == YAML::NodeType::Map) {
		for(YAML::Iterator it = node.begin(); it != node.end(); ++it) {
			std::string groupname;
			it.first() >> groupname;

			Node* group = new Node(groupname);
			
                        root->addChild(group);
			parseYamlNode(it.second(), group);
		}
	} else if(node.Type() == YAML::NodeType::Sequence) {
		for(YAML::Iterator it = node.begin(); it != node.end(); ++it) {
			parseYamlNode(*it, root);
		}
	} else {
		std::cerr << "Failure in map yaml parsing." << std::endl;
		BOOST_ASSERT(0);
	}
}


Environment NodeMap::getEnvironment()
{
    Environment env;

    env.right_bottom_corner = translation;
    env.left_top_corner = limitations + translation;

    std::vector<Node*> leafs = root->getLeafs();
    Landmark landmark;
    Plane plane;
    double height;


    for(unsigned i = 0; i < leafs.size(); i++) {
        switch(leafs[i]->getNodeType()) {
        case NODE_LANDMARK:
    	    landmark.caption = leafs[i]->getCaption();
	    landmark.point = dynamic_cast<LandmarkNode*>(leafs[i])->point();

            env.landmarks.push_back(landmark);
	    break;

        case NODE_LINE:
            height = dynamic_cast<LineNode*>(leafs[i])->getHeight();
	    plane.position  = dynamic_cast<LineNode*>(leafs[i])->getLine().from();
	    plane.span_horizontal = dynamic_cast<LineNode*>(leafs[i])->getLine().to() - plane.position;
	    plane.span_vertical = base::Vector3d(0.0, 0.0, height); 

            env.planes.push_back(plane);

	    break;
	     
        case NODE_BOX:
        {
            Eigen::Vector3d box_position = dynamic_cast<BoxNode*>(leafs[i])->getPosition();
            Eigen::Vector3d box_span = dynamic_cast<BoxNode*>(leafs[i])->getSpan();
            
            plane.position = Eigen::Vector3d(box_position[0] + (box_span[0] * 0.5), box_position[1] + (box_span[1] * 0.5),
                                              box_position[2] + (box_span[2] * 0.5) );
            plane.span_horizontal = Eigen::Vector3d(-box_span[0], 0.0, 0.0);                                  
            plane.span_vertical = Eigen::Vector3d(0.0, 0.0, -box_span[2]);
            env.planes.push_back(plane);
            
            plane.position = Eigen::Vector3d(box_position[0] + (box_span[0] * 0.5), box_position[1] + (box_span[1] * 0.5),
                                              box_position[2] + (box_span[2] * 0.5) );
            plane.span_horizontal = Eigen::Vector3d(0.0, -box_span[1], 0.0);               
            env.planes.push_back(plane);
            
            plane.position = Eigen::Vector3d(box_position[0] - (box_span[0] * 0.5), box_position[1] - (box_span[1] * 0.5),
                                              box_position[2] + (box_span[2] * 0.5) );
            plane.span_horizontal = Eigen::Vector3d(0.0, box_span[1], 0.0);               
            env.planes.push_back(plane);
            
            plane.position = Eigen::Vector3d(box_position[0] - (box_span[0] * 0.5), box_position[1] - (box_span[1] * 0.5),
                                              box_position[2] + (box_span[2] * 0.5) );
            plane.span_horizontal = Eigen::Vector3d(box_span[0], 0.0, 0.0);               
            env.planes.push_back(plane);    

        }
          
          break;
            
        default:
	    break;
        }
    }

    return env;
}


}
