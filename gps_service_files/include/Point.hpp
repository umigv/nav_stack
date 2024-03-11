#pragma once
#include "GPSCoordinate.hpp"
#include <cmath>

class Point{
    public:
    constexpr Point() = default;

    constexpr Point(long double x, long double y) : x(x), y(y){}

    constexpr Point(long double magnitude, long double theta) : x(magnitude * cos(theta)), y(magnitude * sin(theta)){}

    Point(const GPSCoordinate& origin, const GPSCoordinate& destination){
        x = GPSCoordinate::distanceBetween(origin, GPSCoordinate(origin.getLatitude(), destination.getLongitude()));
        y = GPSCoordinate::distanceBetween(origin, GPSCoordinate(destination.getLatitude(), origin.getLongitude()));
    }

    Point(const Point& rhs) : x(rhs.x), y(rhs.y){}

    inline long double getX() const{
        return x;
    }

    inline long double getY() const{
        return y;
    }

    inline void setX(const long double x){
        this->x = x;
    }

    inline void setY(const long double y){
        this->y = y;
    }

    void operator=(const Point& rhs);

    Point operator+(const Point& rhs) const;

    Point operator-(const Point& rhs) const;

    Point operator-() const;

    Point operator*(long double scalar) const;

    Point operator/(long double scalar) const;

    bool operator==(const Point& rhs) const;

    bool operator!=(const Point& rhs) const;

    double theta() const;

    double magnitude() const;

    double distanceTo(const Point& rhs) const;

    Point norm() const;

    Point rotateBy(long double theta) const;

    private:
    long double x{0.0};
    long double y{0.0};
};