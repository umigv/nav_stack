#pragma once
#include <iostream>
#include <cmath>

class GPSCoordinate{
public:
    GPSCoordinate() = default;

    constexpr GPSCoordinate(long double latitude, long double longitude) : latitude(latitude), longitude(longitude){}

    inline long double getLatitude() const{
        return latitude;
    }

    inline long double getLongitude() const{
        return longitude;
    }

    inline void setLatitude(const double long latitude){
        this->latitude = latitude;
    }

    inline void setLongitude(const double long longitude){
        this->longitude = longitude;
    }

    static long double distanceBetween(const GPSCoordinate& current, const GPSCoordinate& target);

private:
    friend std::istream& operator>>(std::istream& is, GPSCoordinate& gpsCoordinate);

    friend std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate);

    long double latitude;
    long double longitude;
    static const constexpr long double TERRESTRIAL_RADIUS_METER{6372797.56085};
    static const constexpr long double RADIANS_PER_DEGREE{M_PI / 180};
};

std::istream& operator>>(std::istream& is, GPSCoordinate& gpsCoordinate);

std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate);