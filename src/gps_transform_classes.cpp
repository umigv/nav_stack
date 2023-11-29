#include "gps_transform_classes.hpp"

using std::atan2;
using std::cos;
using std::pair;
using std::sin;
using std::sqrt;
using std::string;

const bool DEBUG = true;

constexpr double long PI = 3.14159265358979323846;
constexpr double long TERRESTRIAL_RADIUS = 6372797.56085;
constexpr double long RADIANS_PER_DEGREE = PI / 180;

double long Coordinate::getLatitude() const {
    return latitude;
}

double long Coordinate::getLongitude() const {
    return longitude;
}

void Coordinate::setLatitude(double long latitude) {
    this->latitude = latitude;
}

void Coordinate::setLongitude(double long longitude) {
    this->longitude = longitude;
}

double long Coordinate::distanceBetweenPoints(const Coordinate &current, const Coordinate &target) {
    double long latNew = target.getLatitude() * RADIANS_PER_DEGREE;
    double long latOld = current.getLatitude() * RADIANS_PER_DEGREE;
    double long latDiff = (target.getLatitude() - current.getLatitude()) * RADIANS_PER_DEGREE;
    double long lngDiff = (target.getLongitude() - current.getLongitude()) * RADIANS_PER_DEGREE;

    double long a =
            sin(latDiff / 2) * sin(latDiff / 2) + cos(latNew) * cos(latOld) * sin(lngDiff / 2) * sin(lngDiff / 2);
    double long c = 2 * atan2(sqrt(a), sqrt(1 - a));

    double long distance = TERRESTRIAL_RADIUS * c;

    return distance;
}

CoordinateMap CoordinateMap::operator-(const CoordinateMap &other) const {
    return {this->x - other.x, this->y - other.y};
}

CoordinateMap CoordinateMap::operator+(const CoordinateMap &other) const {
    return {this->x + other.x, this->y + other.y};
}

double CoordinateMap::getX() const {
    return x;
}

double CoordinateMap::getY() const {
    return y;
}

void OccGridInfo::update(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN) {
    this->resolution = resolutionIn;
    this->width = widthIn;
    this->height = heightIn;
    this->origin_x = originXIn;
    this->origin_y = originYIN;
}

GPSdata::GPSdata() {
    readConfigFile();
}

void GPSdata::setRobotCurrentLocation(double long lat, double long lon) {
    robCurrentLocation.setLatitude(lat);
    robCurrentLocation.setLongitude(lon);
}

void GPSdata::readGPSFile(const string &filename) {
    double long lat = NAN;
    double long lon = NAN;
    std::ifstream gpsFile(filename);

    if (gpsFile.is_open()) {
        while (gpsFile >> lat >> lon) {
            GOAL_GPS.emplace_back(lat, lon);
        }
    }
}

void GPSdata::readConfigFile() {
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

Coordinate GPSdata::gpsTransform(Coordinate goalPoint) const {
    // TODO: Convert to map coordinate that then gets sent to the planner
    // Can include the vector math or be done in another function

    Coordinate xPoint(robCurrentLocation.getLatitude(), goalPoint.getLongitude());
    Coordinate yPoint(goalPoint.getLatitude(), robCurrentLocation.getLongitude());

    double long xDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, xPoint);
    double long yDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, yPoint);

    // Check for positive and Negative X and Y for northern hemisphere
    if (robCurrentLocation.getLatitude() > goalPoint.getLatitude()) {
        yDistance = fabs(yDistance) * -1;
    } else {
        yDistance = fabs(yDistance);
    }
    if (robCurrentLocation.getLongitude() > goalPoint.getLongitude()) {
        xDistance = fabs(xDistance) * -1;
    } else {
        xDistance = fabs(xDistance);
    }

    if (!north) {
        xDistance *= -1;
        yDistance *= -1;
    }


    return {xDistance, yDistance};
}

bool GPSdata::goalReached(Coordinate &goalCoords) const {
    double long dist = Coordinate::distanceBetweenPoints(goalCoords, robCurrentLocation);
    return dist < 2;
}

CoordinateMap GPSdata::findGoalInMap(MapInfo mapInfo, CoordinateMap goalCoordinate) const {
    // Occupancy Grid corners
    Corner bottomLeft = Corner(mapInfo.origin_x, mapInfo.origin_y);
    Corner topLeft = Corner(mapInfo.origin_x, mapInfo.origin_y + (mapInfo.height * mapInfo.resolution));
    Corner bottomRight = Corner(mapInfo.origin_x + (mapInfo.width * mapInfo.resolution), mapInfo.origin_y);
    // Corner topRight = Corner(bottomRight.x, topLeft.y);

    // Goal Position in global frame
    double goal_x = goalCoordinate.getX();
    double goal_y = goalCoordinate.getY();

    // Goal position inside map
    double return_x = goalCoordinate.getX();
    double return_y = goalCoordinate.getY();

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

    return {return_x, return_y};
}

// int main() {
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
