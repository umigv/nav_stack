#include "../include/coordinate.hpp"
#include <cmath>
#include <string>

using std::atan2;
using std::cos;
using std::sin;
using std::sqrt;
using std::string;

constexpr double long PI = 3.14159265358979323846;
constexpr double long TERRESTRIAL_RADIUS = 6372797.56085;
constexpr double long RADIANS_PER_DEGREE = PI / 180;

double long Coordinate::getLatitude() const
{
    return latitude;
}

double long Coordinate::getLongitude() const
{
    return longitude;
}

void Coordinate::setLatitude(double long latitude)
{
    this->latitude = latitude;
}

void Coordinate::setLongitude(double long longitude)
{
    this->longitude = longitude;
}

double long Coordinate::distanceBetweenPoints(const Coordinate& current, const Coordinate& target)
{
    double long latNew = target.getLatitude() * RADIANS_PER_DEGREE;
    double long latOld = current.getLatitude() * RADIANS_PER_DEGREE;
    double long latDiff = (target.getLatitude() - current.getLatitude()) * RADIANS_PER_DEGREE;
    double long lngDiff = (target.getLongitude() - current.getLongitude()) * RADIANS_PER_DEGREE;

    double long a = sin(latDiff / 2) * sin(latDiff / 2) + cos(latNew) * cos(latOld) * sin(lngDiff / 2) * sin(lngDiff / 2);
    double long c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double long distance = TERRESTRIAL_RADIUS * c;

    return distance;
}

CoordinateMap CoordinateMap::operator-(const CoordinateMap& other) const
{
    return { this->x - other.x, this->y - other.y };
}

CoordinateMap CoordinateMap::operator+(const CoordinateMap& other) const
{
    return { this->x + other.x, this->y + other.y };
}

double CoordinateMap::getX() const
{
    return x;
}

double CoordinateMap::getY() const
{
    return y;
}