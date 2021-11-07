#include <vector>
#include <queue>
#include <numeric>
#include "percep_utilities.hpp"
#include "occupancy_mapping.hpp"

//Do cmd and f and type "changes". Everything I have changed you can find there
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

    CellIndex adjacentCell;
    vector<bool> v;
    queue<int> q;
    queue<int> visited;
    OccupancyMap submap; //create a proper submap

    q.push(robotCell);
    v[robotCell] = true;
    
    while (!q.empty()) //stop when it build a 100x100 map
    {
        int current = q.front();
        q.pop(); 
        

        const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
        const int yDeltas[8] = {0, 1, -1, -1, 1, 0 , -1, 1 };
        for(int i=0; i<8; i++){
            if(isCellInGrid(map.x + xDeltas[i], map.y + yDeltas[i])){
                adjacentCell.colIndex = map.colIndex + xDeltas[i]; //get the cells from the map
                adjacentCell.rowIndex = map.rowIndex + yDeltas[i];
                if(adjacentCell is not in q already){//missing
                    submap.x = adjacentCell.colIndex; //dependent on the submap initialisation
                    submap.y = adjacentCell.rowIndex;
                    submap[x][y] = logOdds[adjacentCell.colIndex][adjacentCell.rowIndex]
                    q.push(adjacentCell);

                }

            }
        
    }
    }

    for(i=0;i<sizeof(obstacles)-1, i++){
        //increase cell odds of all four nodes

        while(a.x != b.x + 1){ //increase the cell odds of all cells within the obstacle
            while(a.y != c.y - 1){
            increaseCellOdds(a.x, a.y, submap);
            a.y = a.y -1;
            }
            a.x = a.x + 1;
        }
    }
    //Decrease cell odds of all other cells that are not occupied
    for(i=0; i <= submap.size()-1;i++){
        for(j=0; j <= submap.[0].size()-1;j++){
            if(logOdds(i,j) < 0){
                decreaseCellOdds(i,j, submap);
            }
        }
    }

    for(i=0; i <= submap.size()-1;i++){
        for(j=0; j <= submap.[0].size()-1;j++){

            
        }
    }

    }
    
    return;
}

//Changes the log odd value in the specificed cell
/*
void OccupancyMap::updateOccupancyValues(std::size_t& xIndex, std::size_t& yIndex, bool occupied) {
    if (occupied) {
        occupancyMap[xIndex][yIndex] = occupancyMap[xIndex][yIndex] + (LOG_ODDS_OCCUPIED/LOG_ODDS_UNOCCUPIED);
    }
    else {
        occupancyMap[xIndex][yIndex] = occupancyMap[xIndex][yIndex] + (LOG_ODDS_UNOCCUPIED/LOG_ODDS_OCCUPIED);
    }
}*/

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
    if (lowerFOV < 0.0) {
        lowerFOV = lowerFOV + 360.0;
    }
}

//Search surrounding Cell and updates their occupancy
void searchCells(std::vector<std::vector<int> >& roverFrame, std::queue<CellIndex>& sreach, CellIndex& current) {
    CellIndex adjacentCell;
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 0 , -1, 1 };
    for(int i=0; i<8; i++){
        adjacentCell.colIndex = current.colIndex + xDeltas[i];
        adjacentCell.rowIndex = current.rowIndex + yDeltas[i];
        float angle = tan(estimateNoneuclid(adjacentCell.rowIndex, roverYPos)/estimateNoneuclid(adjacentCell.colIndex, roverXPos));
        if (angle < upperFOV && angle > lowerFOV) {
            if (roverFrame[adjacentCell.rowIndex][adjacentCell.colIndex] != 1) {
                roverFrame[adjacentCell.rowIndex][adjacentCell.colIndex] = 1;
                search.push(adjacentCell);
            }
        }
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

bool OccupancyGrid::isCellInGrid(int x, int y) const
{ 
    bool xCoordIsValid = (x >= 0) && (x < DEFAULT_OCCUPANCY_MAP_WIDTH);
    bool yCoordIsValid = (y >= 0) && (y < DEFAULT_OCCUPANCY_MAP_HEIGHT);
    return xCoordIsValid && yCoordIsValid;
}

void OccupancyGrid::setLogOdds(int x, int y, CellOdds value)
{
    if(isCellInGrid(x, y))
    {
        OccupancyMap[x][y] = value;
    }
}
CellOdds OccupancyGrid::logOdds(int x, int y) const
{
    if(isCellInGrid(x, y))
    {
        return OccupancyMap[x][y];
    }
    
    return 0;
}

void OccupancyMap::decreaseCellOdds(int x, int y, OccupancyMap& map){
    if(std::numeric_limits<CellOdds>::min() < map(x, y) -kMissOdds_){
    map[x][y] -= kMissOdds_;
    }
    else{
        map[x][y] = std::numeric_limits<CellOdds>::min();
    }

void OccupancyMap::increaseCellOdds(int x, int y, OccupancyMap& map){
    
    if(std::numeric_limits<CellOdds>::max() - map(x,y) > kHitOdds_){
    map[x][y] += kHitOdds_;
    }
    else{
        map[x][y] = std::numeric_limits<CellOdds>::max(); //might be []
    }


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
