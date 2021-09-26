#ifndef PERCEP_OCUPPANCY_MAPPING
#define PERCEP_OCUPPANCY_MAPPING

#include <vector>

const std::size_t DEFAULT_OCCUPANCY_MAP_HEIGHT = 10000;
const std::size_t DEFAULT_OCCUPANCY_MAP_WIDTH = 10000;
const float LOG_ODDS_OCCUPIED = 0.65;
const float LOG_ODDS_UNOCCUPIED = 0.35; // 1 - LOG_ODDS_OCCUPIED

const double ZED_FOV = 90.0; //deg

const double CELL_DISTANCE = 0.4; //0.4 meters
const int MAX_FILTER_LENGTH = 7; //7 meters

//struct to make it easier to index into specific cells of the 2d array
struct CellIndex {
    std::size_t row_index;
    std::size_t col_index;
};

//Struct used when reading in odometery data from a text file
struct Odometry {
    int latitude_deg;
    double latitude_min;
    int longitude_deg;
    double longitude_min;
    double bearing_deg;
    double speed;
};

//define struct for obstacle bounding box
struct Obstacle {
    CellIndex lower;
    CellIndex left;
    CellIndex right;
    CellIndex upper;
};

class OccupancyMap {
private:
    //Vector of vectors that stores the log odds of occupacy for each cell as a float.
    std::vector<std::vector<float> > occupancyMap;
    //Vector which tracks the amount of times each cell has been view on the occupancy map.
    std::vector<std::vector<int> > occupancyMapIteration;

    std::vector<Odometry> odomData;
    std::vector<int> imageTimeData;

    int roverXPos, roverYPos;

    void OccupancyMapViewer();

    void getOdomData();

    void updateRoverPosition();

    void updateOccupancyMap();

    void updateOccupancyValues (std::size_t xIndex, std::size_t yIndex, bool &occupied);

public:
    //Default Constructor for OccupancyMap Class, resizes the vector of vectors to the default map height and width.
    OccupancyMap();

    //Overloaded Constructor for OccupancyMap Class, resizes the vector of vectors to the inputed height and width.
    OccupancyMap(int length, int width);

    void updateMap();
};

#endif
