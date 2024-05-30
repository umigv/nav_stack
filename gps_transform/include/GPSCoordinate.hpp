#pragma once
#include <iostream>
#include <cmath>

/**
 * @brief A class to represent a GPS coordinate. 
 * 
 * The longitude and latitude are stored in radians. The coordinates are stored in long double to prevent overflow.
*/
class GPSCoordinate{
public:
    /**
     * @brief Construct a new GPSCoordinate object
    */
    GPSCoordinate() = default;

    /**
     * @brief Construct a new GPSCoordinate object
     * 
     * @param latitude The latitude of the GPS coordinate in radians
     * @param longitude The longitude of the GPS coordinate in radians
    */
    constexpr GPSCoordinate(long double latitude, long double longitude) : latitude(latitude), longitude(longitude){}

    /**
     * @brief Get the latitude of the GPS coordinate
    */
    inline long double getLatitude() const{
        return latitude;
    }

    /**
     * @brief Get the longitude of the GPS coordinate
    */
    inline long double getLongitude() const{
        return longitude;
    }

    /**
     * @brief Set the latitude of the GPS coordinate
     * 
     * @param latitude The latitude of the GPS coordinate in radians
    */
    inline void setLatitude(const double long latitude){
        this->latitude = latitude;
    }

    /**
     * @brief Set the longitude of the GPS coordinate
     * 
     * @param longitude The longitude of the GPS coordinate in radians
    */
    inline void setLongitude(const double long longitude){
        this->longitude = longitude;
    }

    /**
     * @brief Calculate the absolute distance between two GPS coordinates in meters
     * 
     * This is calculated using the haversine formula (https://en.wikipedia.org/wiki/Haversine_formula).
     * 
     * @param current The current GPS coordinate
     * @param target The target GPS coordinate
    */
    static long double distanceBetween(const GPSCoordinate& current, const GPSCoordinate& target);

private:
    friend std::istream& operator>>(std::istream& is, GPSCoordinate& gpsCoordinate);

    friend std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate);

    long double latitude;
    long double longitude;
    static const constexpr long double TERRESTRIAL_RADIUS_METER{6372797.56085};
    static const constexpr long double RADIANS_PER_DEGREE{M_PI / 180};
};

/**
 * @brief Reads a GPSCoordinate from an input stream in format "latitude longitude"
*/
std::istream& operator>>(std::istream& is, GPSCoordinate& gpsCoordinate);

/**
 * @brief Writes a GPSCoordinate to an output stream in format "(latitude, longitude)"
*/
std::ostream& operator<<(std::ostream& os, const GPSCoordinate& gpsCoordinate);