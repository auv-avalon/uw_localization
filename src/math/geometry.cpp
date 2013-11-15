#include "geometry.hpp"

namespace uw_localization {

Line Line::fromTwoPoints(const Point& p, const Point& q)
{
    return Line(p, q);
}

Line Line::fromPointDirection(const Point& p, const Direction& q)
{
    return Line(p, q + p);
}

Line::Line(const Point& p, const Point &q)
    : p(p), q(q)
{}


Line::Line(const Line& line) 
    : p(line.p), q(line.q)
{}

Line::Line()
{}


Point Line::point(double lambda) const
{
    if(lambda == INFINITY)
        return Point(INFINITY, INFINITY, INFINITY);

    return p + direction() * lambda;
}


Point Line::intersection(const Line& line) const
{
    return point( lambda(line) );
}


Point Line::intersection(const Point& pt) const
{
    return point( lambda(pt) );
}

Point Line::intersectionPoint(const Line& line) const{
  
  //Intersection point
  Point intersection;
  
  //compensation for lack of floating point precission
  double epsilon = 0.01;
  
  //Lineintersection-Algorithm from http://en.wikipedia.org/wiki/Line-line_intersection
  //transform points into A1x + B1y = C1 form
  double A1 = q[1] - p[1];
  double B1 = p[0] - q[0];
  double C1 = A1 * p[0] + B1 * p[1];

  double A2 = line.to()[1] - line.from()[1];
  double B2 = line.from()[0] - line.to()[0];
  double C2 = A2 * line.from()[0] + B2 * line.from()[1];
  
  double determinant = A1 * B2 - A2 * B1;

  if (determinant == 0) {
  //lines are parallel
    return base::Vector3d::Constant(NAN);
  } else {
    //calculate the intersection point
	intersection[0] = (B2 * C1 - B1 * C2) / determinant;
	intersection[1] = (A1 * C2 - A2 * C1) / determinant;
	intersection[2] = 0.0;
  }

  //make sure the intersection point is on both lines
  if ((intersection[0] - fmin(p[0], q[0]) > -epsilon
	&& fmax(p[0], q[0]) - intersection[0] > -epsilon
	&& intersection[1] - fmin(p[1], q[1]) > -epsilon
	&& fmax(p[1], q[1]) - intersection[1] > -epsilon
	&& intersection[0] - fmin(line.from()[0], line.to()[0]) > -epsilon
	&& fmax(line.from()[0], line.to()[0]) - intersection[0] > -epsilon
	&& intersection[1] - fmin(line.from()[1], line.to()[1]) > -epsilon
	&& fmax(line.from()[1], line.to()[1]) - intersection[1] > -epsilon)){
	return intersection;
  }

  return base::Vector3d::Constant(NAN);
}   
 


double Line::lambda(const Point& pt) const
{
    const Direction& r = direction();

    return (r.transpose() * (pt - p)) * (r.transpose() * r).inverse();
}

double Line::distance(const Point& pt) const
{
  Point p2pt = p-pt;
  double alpha = std::acos( (p2pt.dot(direction()) ) / (p2pt.norm() * direction().norm()) );
  return sin(alpha) * p2pt.norm();
}

double Line::lambda(const Line& line) const
{
    const Direction& r = direction();
    const Point& p = from();

    base::Matrix2d A;
    base::Vector2d b;

    A << r.x(), line.direction().x(), r.y(), line.direction().y();
    b << line.from().x() - p.x(), line.from().y() - p.y();

    if(A.determinant() == 0)
        return INFINITY;
    
    return (A.inverse() * b).x();
}


} // namespace uw_localization;
