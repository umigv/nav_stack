#include "../include/MapFrame.hpp"

MapFrame::MapFrame(const Point& origin, uint32_t width, uint32_t height, long double resolution){
    const long double dx = width * resolution;
    const long double dy = height * resolution;

    this->width = dx;
    this->height = dy;
    bottomLeft = origin;
    bottomRight = bottomLeft + Point(dx, 0);
    topLeft = bottomLeft + Point(0, dy);
    topRight = bottomLeft + Point(dx, dy);
} 

Point MapFrame::constrainToMap(const Point& point) const{
    Point ret(point);

    if(dx < 2 * MAP_CONSTRAINT_OFFSET){
        ret.setX(bottomLeft.getX());
    }

    if(dy < 2 * MAP_CONSTRAINT_OFFSET){
        ret.setY(bottomLeft.getY());
    }

    ret.setX(std::max(bottomLeft.getX() + MAP_CONTRAINT_OFFSET, ret.getX()));
    ret.setY(std::max(bottomLeft.getY() + MAP_CONTRAINT_OFFSET, ret.getY()));
    ret.setX(std::min(topRight.getX() - MAP_CONTRAINT_OFFSET, ret.getX()));
    ret.setY(std::min(topRight.getY() - MAP_CONTRAINT_OFFSET, ret.getY()));

    return ret;
}

