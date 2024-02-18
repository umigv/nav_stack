#pragma once

#include <deque>
#include <string>

#include "coordinate.hpp"
// Map Metadata Struct

class GPSData
{
public:
    GPSData(const std::string& nodeNameIn);


    /*
        Sets the current robot position in latitude and longitude
        to the values given by the function arguments.
    */
    void setRobotCurrentLocation(double long lat, double long lon);


    /*
        Read in a text file filled with 6 GPS goal coordinates.
        These GPS goal cordinates come from: 
            gps_service_files/config/waypoints.txt
    */
    void readGPSFile(const std::string& filename);
    
    /*
        Prints out latitude, longitute of all goal GPSData points
        to the terminal.
    */
    void printGPSData() const;  


    /*
        Reads in the config file provided by filename.
        Currently sets value of facing north boolean.
        May want to use parameter server instead.    
    */
    void readConfigFile(const std::string& filename);

    /*
        Given a GPS latitude, longitute coordinate, converts
        the coordinate to an (x, y) coordinate in the map frame.
    */
    Coordinate gpsTransform(const Coordinate& goalPoint) const;


    /*
        Given a (x, y) coordinate in the map frame, checks if the
        robot is within 2 meters of the given goal coordinate.
    */
    bool goalReached(const Coordinate& goalCoords) const;

    void initializeMapInfo(const std::string& configFilename, 
        const bool facingNorthIn); 

    struct MapInfo
    {
        float resolution {0};
        uint32_t width {0};
        uint32_t height {0};
        float origin_x {0};
        float origin_y {0};
    };

    struct Corner
    {
        Corner(double xIn, double yIn) 
        : x(xIn), y(yIn)
        {
        }

        const double x;
        const double y;
    };

    // Inputs:
    //            MapInfo: resolution, width, height, origin_x, origin_y
    //            Goal Coordinate: x, y -- Assumed to be in meters as displacement from global
    //            frame origin
    CoordinateMap findGoalInMap(const CoordinateMap& goalCoordinate) const;

private:
    // LATITUDE, LONGITUDE
    std::deque<Coordinate> waypoints;

    // TODO: tf transform function: NavSatFix
    Coordinate robCurrentLocation;

    // nodeName is the name of the node running this constructor; used for logging
    std::string nodeName;

    uint32_t indexOfCurrentGoal;
    MapInfo mapInfo;
    bool facingNorth;
};