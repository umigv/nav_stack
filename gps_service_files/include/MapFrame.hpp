#pragma once
#include "Point.hpp"

class MapFrame{
    public:
    MapFrame() = default;
    
    MapFrame(const Point& origin, long double width, long double height, long double resolution);

    MapFrame(const MapFrame& rhs);

    MapFrame& operator=(const MapFrame& rhs);

    Point constrainToMap(const Point& point) const;

    private:
    Point bottomLeft;
    Point bottomRight;
    Point topLeft;
    Point topRight;
};
