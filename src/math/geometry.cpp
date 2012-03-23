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


Direction Line::direction() const 
{
    return q - p;
}


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


double Line::lambda(const Point& pt) const
{
    const Direction& r = direction();

    return (r.transpose() * (pt - p)) * (r.transpose() * r).inverse();
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
