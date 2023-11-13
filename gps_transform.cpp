#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <math.h>

using std::pair;
using std::string;

const bool DEBUG = true;

// Coordinate Struct
struct Coordinate {
    Coordinate(double lat, double lon)
        : latitude(lat)
        , longitude(lon) {}
    double latitude;
    double longitude;
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
    //  Final queue of x, y values sent to global frame
    std::deque<Coordinate> GOAL_POINTS;   // This
    std::deque<Coordinate> GOAL_GPS;

    // TODO: tf transform function: NavSatFix
    Coordinate robCurrentLocation { 0, 0 };

    uint32_t indexOfCurrentGoal = 1;
    OccGridInfo mapInfo;
    bool north = false;

    // Read Text File of GPS Coordinates
    void readGPSFile(const string& filename) {
        double lat = NAN;
        double lon = NAN;
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
    auto gpsTransform(Coordinate original) -> Coordinate {
        // TODO: Convert to map coordinate
        return { original.latitude, original.longitude };
    }

    auto goalReached(Coordinate& goalCoords) -> bool {
        double dist = distanceBetweenPoints(goalCoords, robCurrentLocation);
        return dist < 3;
    }

private:
    static auto distanceBetweenPoints(Coordinate& current, Coordinate& target) -> double {
        // haversine formula
        double R = 6371e3;                               // meters
        double phi1 = current.latitude * M_PI / 180.0;   // φ, λ in radians
        double phi2 = target.latitude * M_PI / 180.0;

        double deltaPhi = (target.latitude - current.latitude) * M_PI / 180.0;
        double deltaLambda = (target.longitude - current.longitude) * M_PI / 180.0;

        double a
          = sin(deltaPhi / 2) * sin(deltaPhi / 2) + cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);

        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        double d = R * c;   // in meters

        return d;
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
