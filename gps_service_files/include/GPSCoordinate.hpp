#pragma once

class GPSCoordinate{
public:
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
    friend std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate);

    long double latitude;
    long double longitude;

    static const long double TERRESTRIAL_RADIUS_METER{6372797.56085};
    static const long double RADIANS_PER_DEGREE{3.14159265358979323846 / 180};
};

std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate);