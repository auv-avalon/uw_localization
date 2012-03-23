#ifndef UW_LOCALIZATION__MATH_H
#define UW_LOCALIZATION__MATH_H

#include <base/eigen.h>

namespace uw_localization {

typedef base::Vector3d Point;
typedef base::Vector3d Direction;

// define a line depending on two points
struct Line {
 public:
    static Line fromTwoPoints(const Point& p, const Point& q);
    static Line fromPointDirection(const Point& p, const Direction& q);

    Line(const Line& line);

    inline Direction direction() const;
    inline Point from() const { return p; }
    inline Point to() const { return q; }

    Point point(double lambda) const;

    Point intersection(const Line& line) const;
    Point intersection(const Point& point) const;

    double lambda(const Point& point) const;
    double lambda(const Line& line) const;

 private:
    Line(const Point& p, const Point& q);

    base::Vector3d p;  // beginning point
    base::Vector3d q;  // end point 
};

} // namespace uw_localization

#endif
