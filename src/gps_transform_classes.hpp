#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

// Coordinate Class
class Coordinate {
public:
    Coordinate(double long lat, double long lon) : latitude(lat), longitude(lon) {}

    static double long distanceBetweenPoints(const Coordinate &current, const Coordinate &target);

    double long getLatitude() const;
    double long getLongitude() const;

    void setLatitude(double long latitude);
    void setLongitude(double long longitude);

private:
    double long latitude;
    double long longitude;
};

class CoordinateMap {
public:
    CoordinateMap(double x, double y) : x(x), y(y) {}

    CoordinateMap operator-(const CoordinateMap &other) const;
    CoordinateMap operator+(const CoordinateMap &other) const;

    double getX() const;
    double getY() const;

private:
    double x;
    double y;
};


// Map Metadata Struct
struct OccGridInfo {
    OccGridInfo(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN)
        : resolution(resolutionIn), width(widthIn), height(heightIn), origin_x(originXIn), origin_y(originYIN) {}

    OccGridInfo() = default;

    void update(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN);

    float resolution{0};
    uint32_t width{0};
    uint32_t height{0};
    float origin_x{0};
    float origin_y{0};
};

class GPSdata {
public:
    GPSdata();

    void setRobotCurrentLocation(double long lat, double long lon);

    // Read Text File of GPS Coordinates
    void readGPSFile(const std::string &filename);

    // Read Config File
    void readConfigFile();

    // Convert to Map Coordinate
    Coordinate gpsTransform(Coordinate goalPoint) const;

    bool goalReached(Coordinate &goalCoords) const;

    struct MapInfo {
        MapInfo(float resolution, uint32_t width, uint32_t height, float origin_x, float origin_y)
            : resolution(resolution), width(width), height(height), origin_x(origin_x), origin_y(origin_y) {}

        float resolution;
        uint32_t width;
        uint32_t height;
        float origin_x;
        float origin_y;
    };

    struct Corner {
        Corner(double xIn, double yIn) : x(xIn), y(yIn) {}

        const double x;
        const double y;
    };

    // Inputs:
    //      MapInfo: resolution, width, height, origin_x, origin_y
    //      Goal Coordinate: x, y -- Assumed to be in meters as displacement from global
    //      frame origin
    CoordinateMap findGoalInMap(MapInfo mapInfo, CoordinateMap goalCoordinate) const;

private:
    // LATITUDE, LONGITUDE
    std::deque<Coordinate> GOAL_GPS;

    // TODO: tf transform function: NavSatFix
    Coordinate robCurrentLocation{0, 0};

    uint32_t indexOfCurrentGoal = 1;
    OccGridInfo mapInfo;
    bool north = false;
};