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

using std::atan2;
using std::cos;
using std::pair;
using std::sin;
using std::sqrt;
using std::string;

const bool DEBUG = true;

constexpr double long pi = 3.14159265358979323846;
constexpr double long terrestrialRadius = 6372797.56085;
constexpr double long radianDegrees = pi / 180;

// Coordinate Class
class Coordinate {
public:
    Coordinate(double long lat, double long lon)
        : latitude(lat)
        , longitude(lon) {}

    static auto distanceBetweenPoints(Coordinate& current, Coordinate& target) -> double long {
        double long latNew = target.latitude * radianDegrees;
        double long latOld = current.latitude * radianDegrees;
        double long latDiff = (target.latitude - current.latitude) * radianDegrees;
        double long lngDiff = (target.longitude - current.longitude) * radianDegrees;

        double long a
          = sin(latDiff / 2) * sin(latDiff / 2) + cos(latNew) * cos(latOld) * sin(lngDiff / 2) * sin(lngDiff / 2);
        double long c = 2 * atan2(sqrt(a), sqrt(1 - a));

        double long distance = terrestrialRadius * c;

        return distance;
    }

    double long latitude = NAN;
    double long longitude = NAN;
};

class CoordinateMap {
    public:
    CoordinateMap(double x, double y)
        : x(x)
        , y(y) {}
    double x;
    double y;

    auto operator-(const CoordinateMap& other) const -> CoordinateMap {
        return { this->x - other.x, this->y - other.y };
    }
    auto operator+(const CoordinateMap& other) const -> CoordinateMap {
        return { this->x + other.x, this->y + other.y };
    }
};


// Map Metadata Struct
struct OccGridInfo {
    OccGridInfo(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN)
        : resolution(resolutionIn)
        , width(widthIn)
        , height(heightIn)
        , origin_x(originXIn)
        , origin_y(originYIN) {}

    OccGridInfo() = default;

    void update(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN) {
        this->resolution = resolutionIn;
        this->width = widthIn;
        this->height = heightIn;
        this->origin_x = originXIn;
        this->origin_y = originYIN;
    }

    float resolution { 0 };
    uint32_t width { 0 };
    uint32_t height { 0 };
    float origin_x { 0 };
    float origin_y { 0 };
};

class GPSdata {
public:
    GPSdata() {
        // Read Config File
        readConfigFile();
    }

    // LATITUDE, LONGITUDE
    std::deque<Coordinate> GOAL_GPS;

    // TODO: tf transform function: NavSatFix
    Coordinate robCurrentLocation { 0, 0 };

    void setRobotCurrentLocation(double long lat, double long lon) {
        robCurrentLocation.latitude = lat;
        robCurrentLocation.longitude = lon;
    }

    uint32_t indexOfCurrentGoal = 1;
    OccGridInfo mapInfo;
    bool north = false;

    // Read Text File of GPS Coordinates
    void readGPSFile(const string& filename) {
        double long lat = NAN;
        double long lon = NAN;
        std::ifstream gpsFile(filename);

        if (gpsFile.is_open()) {
            while (gpsFile >> lat >> lon) {
                GOAL_GPS.emplace_back(lat, lon);
            }
        }
    }

    // Read Config File
    void readConfigFile() {
        std::ifstream configFile("config.yaml");
        string line;

        // Format: isFacingNorth: true
        // Check key and value pair with split of ": "
        if (configFile.is_open()) {
            while (std::getline(configFile, line)) {
                auto split = line.find(": ");
                string key = line.substr(0, split);
                string value = line.substr(split + 2);

                if (key == "isFacingNorth") {
                    north = (value == "true");
                }
            }
        }
        if (DEBUG) {
            std::cout << "isFacingNorth: " << north << '\n';
        }
    }

    // Convert to Map Coordinate
    auto gpsTransform(Coordinate goalPoint) -> Coordinate {
        // TODO: Convert to map coordinate that then gets sent to the planner
        // Can include the vector math or be done in another function

        Coordinate xPoint(robCurrentLocation.latitude, goalPoint.longitude);
        Coordinate yPoint(goalPoint.latitude, robCurrentLocation.longitude);

        double long xDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, xPoint);
        double long yDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, yPoint);

        // Check for positive and Negative X and Y for northern hemisphere
        if (robCurrentLocation.latitude > goalPoint.latitude) {
            yDistance = fabs(yDistance) * -1;
        } else {
            yDistance = fabs(yDistance);
        }
        if (robCurrentLocation.longitude > goalPoint.longitude) {
            xDistance = fabs(xDistance) * -1;
        } else {
            xDistance = fabs(xDistance);
        }

        if (!north) {
            xDistance *= -1;
            yDistance *= -1;
        }


        return { xDistance, yDistance };
    }

    auto goalReached(Coordinate& goalCoords) -> bool {
        double long dist = Coordinate::distanceBetweenPoints(goalCoords, robCurrentLocation);
        return dist < 2;
    }

    struct MapInfo {

        MapInfo(float resolution, uint32_t width, uint32_t height, float origin_x, float origin_y)
            : resolution(resolution)
            , width(width)
            , height(height)
            , origin_x(origin_x)
            , origin_y(origin_y) {}

        float resolution;
        uint32_t width;
        uint32_t height;
        float origin_x;
        float origin_y;

    };

    // Inputs:
    //      MapInfo: resolution, width, height, origin_x, origin_y
    //      Goal Coordinate: x, y -- Assumed to be in meters as displacement from global frame origin
    auto findGoalInMap(MapInfo mapInfo, CoordinateMap goalCoordinate) -> CoordinateMap {

        struct Corner {
            Corner(double xIn, double yIn) : x(xIn), y(yIn) {}
            double x;
            double y;
        };

        // Occupancy Grid corners
        Corner bottomLeft = Corner(mapInfo.origin_x, mapInfo.origin_y);
        Corner topLeft = Corner(mapInfo.origin_x, mapInfo.origin_y + (mapInfo.height * mapInfo.resolution));
        Corner bottomRight = Corner(mapInfo.origin_x + (mapInfo.width * mapInfo.resolution), mapInfo.origin_y);
        // Corner topRight = Corner(bottomRight.x, topLeft.y);

        // Goal Position in global frame
        double goal_x = goalCoordinate.x;
        double goal_y = goalCoordinate.y;

        // Goal position inside map
        double return_x = goalCoordinate.x; 
        double return_y = goalCoordinate.y;

        // Check if goal is outside of map
        if (goal_x < bottomLeft.x) {
            return_x = bottomLeft.x + 0.25;
        } else if (goal_x > bottomRight.x) {
            return_x = bottomRight.x - 0.25;
        }

        if (goal_y < bottomLeft.y) {
            return_y = bottomLeft.y + 0.25;
        } else if (goal_y > topLeft.y) {
            return_y = topLeft.y - 0.25;
        }

        return { return_x, return_y };

    }



};

// auto main() -> int {
//     // Make gpsData instance
//     GPSdata gps;

//     // Read GPS File
//     gps.readGPSFile("gps.txt");

//     // Do Testing
//     if (DEBUG) {
//         std::cout << "GPS Coordinates: \n";
//         for (auto& coord : gps.GOAL_GPS) {
//             std::cout << coord.latitude << ", " << coord.longitude << '\n';
//         }
//     }

//     Coordinate robotLoc = Coordinate(42.29607847274641, -83.70659486161985);

//     gps.setRobotCurrentLocation(robotLoc.latitude, robotLoc.longitude);

//     Coordinate goalLoc = Coordinate(42.29958131982664, -83.70565689495105);

//     // std::cout << "First coordinate (robot location):" << robotLoc.latitude << ", " << robotLoc.longitude << '\n';
//     // std::cout << "Second coordinate (goal location):" << goalLoc.latitude << ", " << goalLoc.longitude << '\n';

//     // print long double robot location and goal location with printf
//     printf("First coordinate (robot location): %Lf, %Lf\n", robotLoc.latitude, robotLoc.longitude);
//     printf("Second coordinate (goal location): %Lf, %Lf\n", goalLoc.latitude, goalLoc.longitude);

//     Coordinate difference = gps.gpsTransform(goalLoc);
//     // std::cout << "Difference (x, y): " << difference.latitude << ", " << difference.longitude << '\n';
//     printf("Difference (x, y): %Lf, %Lf\n", difference.latitude, difference.longitude);


// }
