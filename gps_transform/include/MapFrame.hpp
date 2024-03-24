#pragma once
#include "Point.hpp"

/**
 * @brief A class to represent a frame of reference for the occupancy grid. 
 * 
 * This class is used to constrain points to the map.
*/
class MapFrame{
    public:
    /**
     * @brief Construct a new MapFrame object
    */
    MapFrame() = default;
    
    /**
     * @brief Construct a new MapFrame object
     * 
     * @param origin The origin of the map frame, which is the bottom left corner of the map
     * @param width The width of the map frame in number of cells
     * @param height The height of the map frame in number of cells
     * @param resolution The resolution of the map frame in meters per cell
    */
    MapFrame(const Point& origin, uint32_t width, uint32_t height, long double resolution);

    /**
     * @brief Constrain a point to the map frame
     * 
     * If the point is outside the map frame, it is moved to MAP_CONSTRAINT_OFFSET (0.25) meters within the nearest edge.
    */
    Point constrainToMap(const Point& point) const;

    private:
    Point bottomLeft;
    Point bottomRight;
    Point topLeft;
    Point topRight;
    long double width;
    long double height;

    static constexpr long double MAP_CONTRAINT_OFFSET = 0.25;
};
