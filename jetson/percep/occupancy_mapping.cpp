#include <vector>
#include <queue>

#include "utilities.hpp"
#include "occupancy_mapping.hpp"

// Coverts the input degree (and optional minute) to radians.
double degreeToRadian(const double degree, const double minute) {
    return (PI/180) * (degree + minute/60);
}

// Converts the input radians to degrees.
double radianToDegree(const double radian) {
    return radian * 180/PI;
}

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

void getFOVBounds() {
    upperFOV = heading + (DEFAULT_FOV/2.0);
    if (upperFOV >= 360.0) {
        upperFOV = upperFOV - 360.0;
    }

    lowerFOV = heading - (DEFAULT_FOV/2.0);
    if (upperFOV < 0.0) {
        upperFOV = upperFOV + 360.0;
    }
}

//Search surrounding Cell and updates their occupancy
void searchCells(std::vector<std::vector<int> >& roverFrame, std::queue<CellIndex>& sreach, CellIndex& current) {
    //Check the eight cells arround the current cell
    CellIndex temp;
    temp.colIndex = current.colIndex + 1;
    temp.rowIndex = current.rowIndex + 0;
    float angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

    temp.colIndex = current.colIndex + 1;
    temp.rowIndex = current.rowIndex + 1;
    angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

    temp.colIndex = current.colIndex + 0;
    temp.rowIndex = current.rowIndex + 1;
    angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

    temp.colIndex = current.colIndex - 1;
    temp.rowIndex = current.rowIndex + 0;
    angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

    temp.colIndex = current.colIndex - 1;
    temp.rowIndex = current.rowIndex - 1;
    angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

    temp.colIndex = current.colIndex + 0;
    temp.rowIndex = current.rowIndex - 1;
    angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

    temp.colIndex = current.colIndex - 1;
    temp.rowIndex = current.rowIndex + 1;
    angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

    temp.colIndex = current.colIndex + 1;
    temp.rowIndex = current.rowIndex - 1;
    angle = tan(estimateNoneuclid(temp.rowIndex, roverYPos)/estimateNoneuclid(temp.colIndex, roverXPos));
    if (angle < upperFOV && angle > lowerFOV) {
        if (roverFrame[temp.rowIndex][temp.colIndex] != 1) {
            roverFrame[temp.rowIndex][temp.colIndex] = 1;
            search.push(temp);
        }
    }

}

//Updates the coordinate position and cell position of the rover based on new Odometry data
//TODO: Create a percep utilities.cpp/utilities.hpp to do a lot of these calculations
void updateRoverPosition(const Odometry& newOdomData) {
    double changeInDistanceFromPreviousPosition = estimateNoneuclid(currOdometry, newOdomData);
    double changeInBearingFromPreviousPosition = calcBearing(currOdometry, newOdomData);

    if (changeInBearingFromPreviousPosition < 0) {
        changeInBearingFromPreviousPosition += 360.0;
    }

    //if the bearing is between 0 and 90
    if(changeInBearingFromPreviousPosition =< 90.0) {
        changeInPositionWidth = changeInEuclideanDistanceFromPreviousPosition * cos(90.0 - changeInBearingFromPreviousPosition);
        changeInPositionHeight = changeInEuclideanDistanceFromPreviousPosition * sin(90.0 - changeInBearingFromPreviousPosition);
    }
    //if bearing is between 90 - 180
    else if(changeInBearingFromPreviousPosition > 90.0 && changeInBearingFromPreviousPosition =< 180.0) {
        changeInPositionWidth = cos(changeInBearingFromPreviousPosition - 90.0);
        changeInPositionHeight = sin(changeInBearingFromPreviousPosition - 90.0);
    }
    else if(changeInBearingFromPreviousPosition > 180.0 && changeInBearingFromPreviousPosition =< 270.0) {
        changeInPositionWidth = cos(270.0 - changeInBearingFromPreviousPosition);
        changeInPositionHeight = sin(270.0 - changeInBearingFromPreviousPosition);
    }
    else {
        changeInPositionWidth = cos(changeInBearingFromPreviousPosition - 270.0);
        changeInPositionHeight = sin(hangeInBearingFromPreviousPosition - 270.0);
    }

    roverXPos += changeInPositionWidth;
    roverYPos += changeInPositionHeight;

    roverRowIndex = ceil(changeInPositionWidth/CELL_DISTANCE);
    roverColIndex = ceil(changeInPositionHeight/CELL_DISTANCE);

    //then update odometry
    currOdometry = newOdomData;
}

//Iterated through all cell in the FOV and updated log odds of occupancy
void updateOccupancyMap();  {
    CellIndex start;
    start.colIndex = std::static_cast<std::size_t>(roverXIndex);
    start.rowIndex = std::static_cast<std::size_t>(roverYIndex);

    std::vector<std::vector<int> > roverFrame(100, vector<int>(100, 0));
    loadObtacles(roverFrame);

    std::queue<CellIndex> searcher;
    searcher.push(start);

    while (!searcher.empty()) {
        frontCell = searcher.front();
        searcher.pop();
        search_cells(roverFrame, searcher, frontCell);
    }
}
