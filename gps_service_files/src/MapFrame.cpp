#include "MapFrame.hpp"

MapFrame::MapFrame(const Point& origin, long double width, long double height, long double resolution){
    const long double dx = width * resolution;
    const long double dy = height * resolution;

    bottomLeft = origin;
    bottomRight = bottomLeft + Point(dx, 0);
    topLeft = bottomLeft + Point(0, dy);
    topRight = bottomLeft + Point(dx, dy);
} 

MapFrame::MapFrame(const MapFrame& rhs){
    bottomLeft = rhs.bottomLeft;
    bottomRight = rhs.bottomRight;
    topLeft = rhs.topLeft;
    topRight = rhs.topRight;
}

MapFrame& MapFrame::operator=(const MapFrame& rhs){
    MapFrame temp(rhs);
    std::swap(bottomLeft, temp.bottomLeft);
    std::swap(bottomRight, temp.bottomRight);
    std::swap(topLeft, temp.topLeft);
    std::swap(topRight, temp.topRight);

    return *this;
}

Point MapFrame::constrainPoint(const Point& point) const{
    Point ret(point);

    ret.setX(std::max(bottomLeft.getX() + 0.25, ret.getX()));
    ret.setY(std::max(bottomLeft.getY() + 0.25, ret.getY()));
    ret.setX(std::min(topRight.getX() - 0.25, ret.getX()));
    ret.setY(std::min(topRight.getY() - 0.25, ret.getY()));

    return ret;
}

