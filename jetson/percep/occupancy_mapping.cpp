#include <vector>
#include <queue>

#include "utilities.cpp"
#include "occupancy_mapping.hpp"

OccupancyMap::OccupancyMap(const Odometry &initialOdom) {
    occupancyMap = occupancyMap(DEFAULT_OCCUPANCY_MAP_HEIGHT, std::vector(DEFAULT_OCCUPANCY_MAP_WIDTH, 0.5));
    occupancyMapIteration = occupancyMapIteration(DEFAULT_OCCUPANCY_MAP_HEIGHT, std::vector(DEFAULT_OCCUPANCY_MAP_WIDTH, 0));

    //initialize rover's position in the center
    roverRowIndex = (DEFAULT_OCCUPANCY_MAP_WIDTH/2) - 1;
    roverColIndex = (DEFAULT_OCCUPANCY_MAP_HEIGHT/2) - 1;

    //initialize roverXPos and roverYPos
    roverXPos = 0.0;
    roverYPos = 0.0;

    //initalize Current Odometry Value
    currOdom = initialOdom;
}

//Loads obstacle data in the vector representing the 100 cell x 100 cell region surrounding the rover
void loadObtacles(std::vector<std::vector<int> >& roverFrame /*vector of Obstacle Structs*/) {
    //right now, just assume that there are no obstacles.
    //TODO: Create an interface between the obstacle data and write code to load obstacles into roverFrame vector
    return;
}

//Changes the log odd value in the specificed cell
void OccupancyMap::updateOccupancyValues(std::size_t& xIndex, std::size_t& yIndex, bool occupied) {
    if (occupied) {
        occupancyMap[xIndex][yIndex] = occupancyMap[xIndex][yIndex] + (LOG_ODDS_OCCUPIED/LOG_ODDS_UNOCCUPIED);
    }
    else {
        occupancyMap[xIndex][yIndex] = occupancyMap[xIndex][yIndex] + (LOG_ODDS_UNOCCUPIED/LOG_ODDS_OCCUPIED);
    }
}

//Calls an OpenCV viewer to view a greyscalled 100 cell x 100 cell region of the occupancy map centered at the rover
void viewer() {
    //TODO: Develop this OpenCV viewer code
    return;
}