#include "Point.hpp"

void Point::operator=(const Point& rhs) {
    Point temp(rhs);
    std::swap(x, temp.x);
    std::swap(y, temp.y);
}

Point Point::operator+(const Point& rhs) const {
    return Point(x + rhs.x, y + rhs.y);
}

Point Point::operator-(const Point& rhs) const {
    return -rhs + *this;
}

Point Point::operator-() const {
    return *this * -1;
}

Point Point::operator*(long double scalar) const {
    return Point(x * scalar, y * scalar);
}

Point Point::operator/(long double scalar) const {
    return *this * (1.0 / scalar);
}

bool Point::operator==(const Point& rhs) const {
    return abs(x - rhs.x) < 1E-9 && abs(y - rhs.y) < 1E-9;
}

bool Point::operator!=(const Point& rhs) const {
    return !(*this == rhs);
}

double Point::theta() const {
    return atan2(y, x);
}

double Point::magnitude() const {
    return hypot(x, y);
}

double Point::distanceTo(const Point& rhs) const {
    return (rhs-*this).magnitude();
}

Point Point::rotateBy(long double theta) const {
    return {x * cos(theta) - y * sin(theta), x * sin(theta) + y * cos(theta)};
}

Point Point::norm() const {
    return *this / magnitude();
}