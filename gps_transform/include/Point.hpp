#pragma once

#include "GPSCoordinate.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

/**
 * @brief A class to represent a 2D point. 
 * 
 * The coordinates are stored in long double for consistency with GPS Coordinates.
*/
class Point{
    public:
    /**
     * @brief Construct a new Point object
    */
    Point() = default;

    /**
     * @brief Construct a new Point object
     * 
     * @param x The x coordinate of the point
     * @param y The y coordinate of the point
    */
    Point(long double x, long double y) : x(x), y(y){}

    /**
     * @brief Construct a new Point object from two GPS coordinates
     * 
     * This constructor calculates the x and y coordinates of the point from the origin and destination GPS coordinates.
     * dx and dy is approximated as distance from origin to (origin's longitude, destination's latitude) and 
     * (destination's loogitude, origin's latitude). This method is accurate up to a couple states which is good enough for IGVC.
     * Since the distance is absolute, the direction is determined by the sign of the difference in latitude and longitude.
     * 
     * @param origin The origin GPS coordinate
     * @param destination The destination GPS coordinate
    */
    Point(const GPSCoordinate& origin, const GPSCoordinate& destination);

    /**
     * @brief Get the x coordinate of the point
    */
    inline long double getX() const{
        return x;
    }

    /**
     * @brief Get the y coordinate of the point
    */
    inline long double getY() const{
        return y;
    }

    /**
     * @brief Set the x coordinate of the point
     * 
     * @param x The x coordinate of the point
    */
    inline void setX(const long double x){
        this->x = x;
    }

    /**
     * @brief Set the y coordinate of the point
     * 
     * @param y The y coordinate of the point
    */
    inline void setY(const long double y){
        this->y = y;
    }

    /**
     * @brief Sums this point and another point
     * 
     * @param rhs The point to add
    */
    Point operator+(const Point& rhs) const;

    /**
     * @brief Subtract a point from this point
     * 
     * @param rhs The point to subtract
    */
    Point operator-(const Point& rhs) const;

    /**
     * @brief Negate the point
    */
    Point operator-() const;

    /**
     * @brief Multiply the point by a scalar
     * 
     * @param scalar The scalar to multiply by
    */
    Point operator*(long double scalar) const;

    /**
     * @brief Divide the point by a scalar
     * 
     * @param scalar The scalar to divide by
    */
    Point operator/(long double scalar) const;

    /**
     * @brief Check if two points are equal.
     * 
     * Points are considered equal if the difference in x and y coordinates is less than DIFFERENCE_EPSILON (1e-9)
     * 
     * @param rhs The point to compare to
    */
    bool operator==(const Point& rhs) const;

    /**
     * @brief Check if two points are not equal.
     * 
     * Negates the result of the equality operator.
     * 
     * @param rhs The point to compare to
    */
    bool operator!=(const Point& rhs) const;

    /**
     * @brief Get the angle of the point in radians
    */
    double theta() const;

    /**
     * @brief Get the magnitude of the point
    */
    double magnitude() const;

    /**
     * @brief Get the distance to another point
     * 
     * @param rhs The point to calculate the distance to
    */
    double distanceTo(const Point& rhs) const;

    /**
     * @brief Get the normalized vector of the point
    */
    Point norm() const;

    /**
     * @brief Rotate the point by an angle
     * 
     * @param theta The angle to rotate by in radians
    */
    Point rotateBy(long double theta) const;

    /**
     * @brief Convert the point to a goal for the navigate to pose action
    */
    nav2_msgs::action::NavigateToPose::Goal toNavigateToPoseGoal() const;


    /**
     * @brief Convert the point to a PoseStamped message
    */
    geometry_msgs::msg::PoseStamped toPoseStamped() const;



    private:
    friend std::ostream& operator<<(std::ostream& os, const Point& point);

    long double x{0.0};
    long double y{0.0};
    
    static constexpr long double DIFFERENCE_EPSILON = 1E-9;
};

/**
 * @brief Output a point to an output stream in format "(x, y)"
 * 
 * @param os The output stream
 * @param point The point to output
*/
std::ostream& operator<<(std::ostream& os, const Point& point);