#include <cmath>
#include "../include/Point.hpp"

Point::Point(const GPSCoordinate& origin, const GPSCoordinate& destination){
    x = GPSCoordinate::distanceBetween(origin, GPSCoordinate(origin.getLatitude(), destination.getLongitude()));
    y = GPSCoordinate::distanceBetween(origin, GPSCoordinate(destination.getLatitude(), origin.getLongitude()));

    if(origin.getLatitude() > destination.getLatitude()){
        y *= -1;
    }

    if(origin.getLongitude() > destination.getLongitude()){
        x *= -1;
    }
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
    return abs(x - rhs.x) < DIFFERENCE_EPSILON && abs(y - rhs.y) < DIFFERENCE_EPSILON;
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

nav2_msgs::action::NavigateToPose::Goal Point::toNavigateToPoseGoal() const{
    nav2_msgs::action::NavigateToPose::Goal ret;
    ret.pose = toPoseStamped();

    return ret;
}

geometry_msgs::msg::PoseStamped Point::toPoseStamped() const{
    geometry_msgs::msg::PoseStamped ret;
    ret.header.frame_id = "map";
    ret.pose.position.x = x;
    ret.pose.position.y = y;
    ret.pose.position.z = 0;
    ret.pose.orientation.x = 0;
    ret.pose.orientation.y = 0;
    ret.pose.orientation.z = 0;
    ret.pose.orientation.w = 1;

    return ret;
}


std::ostream& operator<<(std::ostream& os, const Point& point) {
    os << "(" << point.x << ", " << point.y << ")";
    return os;
}