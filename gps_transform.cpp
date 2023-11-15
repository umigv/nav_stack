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
    // TODO: Edit constructor with tfBuffer once working
    GPSdata() {
        // Read Config File
        readConfigFile();
    }

    // LATITUDE, LONGITUDE
    std::deque<Coordinate> GOAL_GPS;

    // TODO: tf transform function: NavSatFix
    Coordinate robCurrentLocation { 0, 0 };

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
        if (robCurrentLocation.latitude < goalPoint.latitude) {
            xDistance *= -1;
        }
        if (robCurrentLocation.longitude < goalPoint.longitude) {
            yDistance *= -1;
        }

        if (north) {
            xDistance *= -1;
            yDistance *= -1;
        }


        return { xDistance, yDistance };
    }

    auto goalReached(Coordinate& goalCoords) -> bool {
        double long dist = Coordinate::distanceBetweenPoints(goalCoords, robCurrentLocation);
        return dist < 2;
    }
};

auto main() -> int {
    // Make gpsData instance
    GPSdata gps;

    // Read GPS File
    gps.readGPSFile("gps.txt");

    // Do Testing
    if (DEBUG) {
        std::cout << "GPS Coordinates: \n";
        for (auto& coord : gps.GOAL_GPS) {
            std::cout << coord.latitude << ", " << coord.longitude << '\n';
        }
    }
}
