#include "../include/GPSCoordinate.hpp"

long double GPSCoordinate::distanceBetween(const GPSCoordinate& current, const GPSCoordinate& target){
    const long double dLatitude = (target.getLatitude() - current.getLatitude()) * RADIANS_PER_DEGREE;
    const long double dLongitude = (target.getLongitude() - current.getLongitude()) * RADIANS_PER_DEGREE;

    const long double a = pow(sin(dLatitude / 2), 2) + pow(sin(dLongitude / 2), 2) * cos(current.getLatitude()) * cos(target.getLatitude());
    const long double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return TERRESTRIAL_RADIUS_METER * c;
}

std::istream& operator>>(std::istream& is, GPSCoordinate& gpsCoordinate){
    is >> gpsCoordinate.latitude >> gpsCoordinate.longitude;
    return is;
}

std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate){
    os << "(" << gpsCoordinate.getLatitude() << ", " << gpsCoordinate.getLongitude() << ")";
    return os;
}