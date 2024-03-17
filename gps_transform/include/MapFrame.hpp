#pragma once
#include "Point.hpp"

class MapFrame{
    public:
    MapFrame() = default;
    
    MapFrame(const Point& origin, long double width, long double height, long double resolution);

    Point constrainToMap(const Point& point) const;

    private:
    Point bottomLeft;
    Point bottomRight;
    Point topLeft;
    Point topRight;

    static constexpr long double MAP_CONTRAINT_OFFSET = 0.25;
};
