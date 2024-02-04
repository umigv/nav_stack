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

const bool DEBUG = true;


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