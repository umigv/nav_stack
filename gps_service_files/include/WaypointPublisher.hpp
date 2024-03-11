#pragma once
#include<iostream>
#include<deque>
#include "Point.hpp"

class WaypointPublisher{
    public:
    WaypointPublisher(std::ifstream& file, bool reversed = false);

    bool isAtWaypoint() const;

    void foo(){
        while(!waypoints.empty()){
            Point targetPoint = constrain(tf + Point(robotGPS, waypoints.front()));

            if(isAtWaypoint()){
                waypoints.pop_front();
            }

            publish(targetPoint)
        }
    }

    private:
    std::deque<GPSCoordinate> waypoints;
    bool reversed;
};