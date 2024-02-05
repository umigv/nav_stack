#include "../include/gps_data.hpp"
#include <cmath>
#include <fstream>
#include <iostream>

using std::string;

void OccGridInfo::update(float resolutionIn, uint32_t widthIn, uint32_t heightIn, float originXIn, float originYIN)
{
  this->resolution = resolutionIn;
  this->width = widthIn;
  this->height = heightIn;
  this->origin_x = originXIn;
  this->origin_y = originYIN;
}

GPSData::GPSData()
{
  readConfigFile();
}

void GPSData::setRobotCurrentLocation(double long lat, double long lon)
{
  robCurrentLocation.setLatitude(lat);
  robCurrentLocation.setLongitude(lon);
}

void GPSData::readGPSFile(const string& filename)
{
  double long lat = NAN;
  double long lon = NAN;
  std::ifstream gpsFile(filename);

  if (gpsFile.is_open())
  {
    while (gpsFile >> lat >> lon)
    {
      GOAL_GPS.emplace_back(lat, lon);
    }
  }
}

void GPSData::printGPSData() const
{
  for (auto& coord : GOAL_GPS)
  {
    std::cout << coord.getLatitude() << ", " << coord.getLongitude() << '\n';
  }
}

void GPSData::readConfigFile()
{
  std::ifstream configFile("config.yaml");
  string line;

  // Format: isFacingNorth: true
  // Check key and value pair with split of ": "
  if (configFile.is_open())
  {
    while (std::getline(configFile, line))
    {
      auto split = line.find(": ");
      string key = line.substr(0, split);
      string value = line.substr(split + 2);

      if (key == "isFacingNorth")
      {
        north = (value == "true");
      }
    }
  }
  if (DEBUG)
  {
    std::cout << "isFacingNorth: " << north << '\n';
  }
}

Coordinate GPSData::gpsTransform(Coordinate goalPoint) const
{
  // TODO: Convert to map coordinate that then gets sent to the planner
  // Can include the vector math or be done in another function

  Coordinate xPoint(robCurrentLocation.getLatitude(), goalPoint.getLongitude());
  Coordinate yPoint(goalPoint.getLatitude(), robCurrentLocation.getLongitude());

  double long xDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, xPoint);
  double long yDistance = Coordinate::distanceBetweenPoints(robCurrentLocation, yPoint);

  // Check for positive and Negative X and Y for northern hemisphere
  if (robCurrentLocation.getLatitude() > goalPoint.getLatitude())
  {
    yDistance = std::abs(yDistance) * -1;
  }
  else
  {
    yDistance = std::abs(yDistance);
  }
  if (robCurrentLocation.getLongitude() > goalPoint.getLongitude())
  {
    xDistance = std::abs(xDistance) * -1;
  }
  else
  {
    xDistance = std::abs(xDistance);
  }

  if (!north)
  {
    xDistance *= -1;
    yDistance *= -1;
  }

  return { xDistance, yDistance };
}

bool GPSData::goalReached(Coordinate& goalCoords) const
{
  double long dist = Coordinate::distanceBetweenPoints(goalCoords, robCurrentLocation);
  return dist < 2;
}

CoordinateMap GPSData::findGoalInMap(MapInfo mapInfo, CoordinateMap goalCoordinate) const
{
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
  if (goal_x < bottomLeft.x)
  {
    return_x = bottomLeft.x + 0.25;
  }
  else if (goal_x > bottomRight.x)
  {
    return_x = bottomRight.x - 0.25;
  }

  if (goal_y < bottomLeft.y)
  {
    return_y = bottomLeft.y + 0.25;
  }
  else if (goal_y > topLeft.y)
  {
    return_y = topLeft.y - 0.25;
  }

  return { return_x, return_y };
}

void GPSData::initializeMapInfo()
{
  readConfigFile();
  readGPSFile("gps.txt");
}

// int main() {
//     // Make GPSData instance
//     GPSData gps;

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