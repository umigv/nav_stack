#include "../include/GPSCoordinate.hpp"

long double GPSCoordinate::distanceBetween(const GPSCoordinate& current, const GPSCoordinate& target){
    const long double latNew = target.getLatitude() * RADIANS_PER_DEGREE;
    const long double latOld = current.getLatitude() * RADIANS_PER_DEGREE;
    const long double latDiff = (target.getLatitude() - current.getLatitude()) * RADIANS_PER_DEGREE;
    const long double lngDiff = (target.getLongitude() - current.getLongitude()) * RADIANS_PER_DEGREE;

    const double long a = sin(latDiff / 2) * sin(latDiff / 2) + cos(latNew) * cos(latOld) * sin(lngDiff / 2) * sin(lngDiff / 2);
    const double long c = 2 * atan2(sqrt(a), sqrt(1 - a));

    const double long distance = TERRESTRIAL_RADIUS_METER * c;

    return distance;
}

std::istream& operator>>(std::istream& is, GPSCoordinate& gpsCoordinate){
    is >> gpsCoordinate.latitude >> gpsCoordinate.longitude;
    return is;
}

std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate){
    os << "(" << gpsCoordinate.getLatitude() << ", " << gpsCoordinate.getLongitude() << ")";
    return os;
}