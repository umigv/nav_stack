#include "rclcpp/rclcpp.hpp"
#include "../include/gps_data.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

GPSData::GPSData(const std::string& nodeNameIn) 
: robCurrentLocation{0, 0}, 
    nodeName(nodeNameIn), 
    indexOfCurrentGoal(0)
{   
    RCLCPP_INFO(rclcpp::get_logger(nodeName), "%s running GPSData constructor", nodeName.c_str());

    // TODO, find out actual mapInfo, get actual robot current location
}

void GPSData::setRobotCurrentLocation(const double long lat, const double long lon)
{
    robCurrentLocation.setLatitude(lat);
    robCurrentLocation.setLongitude(lon);
}

void GPSData::readGPSFile(const std::string& filename)
{
    double long lat = NAN;
    double long lon = NAN;
    std::ifstream gpsFile(filename);

    if(!gpsFile.is_open())
        RCLCPP_WARN(rclcpp::get_logger(nodeName), "%s could not be opened", filename.c_str());

    while (gpsFile >> lat >> lon)
    {
        waypoints.emplace_back(lat, lon);
    }
}

void GPSData::printGPSData() const
{
    for (auto& coord : waypoints)
    {
        RCLCPP_INFO(rclcpp::get_logger(nodeName), 
            "latitude: %Lf, longitude: %Lf\n", coord.getLatitude(), coord.getLongitude());
    }
}

Coordinate GPSData::gpsTransform(const Coordinate& goalPoint) const
{
    // TODO: Convert to map coordinate that then gets sent to the planner
    // Can include the vector math or be done in another function

    Coordinate xPoint(robCurrentLocation.getLatitude(), goalPoint.getLongitude());
    Coordinate yPoint(goalPoint.getLatitude(), robCurrentLocation.getLongitude());

    double long xDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, xPoint);
    double long yDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, yPoint);

    // Check for positive and Negative X and Y for northern hemisphere
    if (robCurrentLocation.getLatitude() > goalPoint.getLatitude())
    {
        yDistance = std::abs(yDistance) * -1;
    }
    else
    {
        yDistance = std::abs(yDistance);
    }
    if (robCurrentLocation.getLongitude() > goalPoint.getLongitude())
    {
        xDistance = std::abs(xDistance) * -1;
    }
    else
    {
        xDistance = std::abs(xDistance);
    }

    if (!facingNorth)
    {
        xDistance *= -1;
        yDistance *= -1;
    }

    return { xDistance, yDistance };
}

bool GPSData::goalReached() const
{
    double long dist = Coordinate::distanceBetweenPoints(getCurrentWaypoint(), robCurrentLocation);
    return dist < 2;
}

CoordinateMap GPSData::findGoalInMap(const CoordinateMap& goalCoordinate) const
{
    // Occupancy Grid corners
    const Corner bottomLeft = Corner(mapInfo.origin_x, mapInfo.origin_y);
    const Corner topLeft = Corner(mapInfo.origin_x, mapInfo.origin_y + (mapInfo.height * mapInfo.resolution));
    const Corner bottomRight = Corner(mapInfo.origin_x + (mapInfo.width * mapInfo.resolution), mapInfo.origin_y);
    // Corner topRight = Corner(bottomRight.x, topLeft.y);

    // Goal Position in global frame
    const double goal_x = goalCoordinate.getX();
    const double goal_y = goalCoordinate.getY();

    // Goal position inside map
    double return_x = goalCoordinate.getX();
    double return_y = goalCoordinate.getY();

    // Check if goal is outside of map
    if (goal_x < bottomLeft.x)
    {
        return_x = bottomLeft.x + 0.25;
    }
    else if (goal_x > bottomRight.x)
    {
        return_x = bottomRight.x - 0.25;
    }

    if (goal_y < bottomLeft.y)
    {
        return_y = bottomLeft.y + 0.25;
    }
    else if (goal_y > topLeft.y)
    {
        return_y = topLeft.y - 0.25;
    }

    return { return_x, return_y };
}

// Will eventually initialize mapInfo as well
void GPSData::initializeMapInfo( const std::string& waypointsFilename, const bool facingNorthIn)
{
    readGPSFile(waypointsFilename);
    facingNorth = facingNorthIn;
}


Coordinate GPSData::getCurrentWaypoint() const
{
    return waypoints[indexOfCurrentGoal];
}