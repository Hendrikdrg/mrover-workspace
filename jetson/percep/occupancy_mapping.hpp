#pragma once

#include <vector>

//TODO: Move these constants to a config file
const std::size_t DEFAULT_OCCUPANCY_MAP_HEIGHT = 10000;
const std::size_t DEFAULT_OCCUPANCY_MAP_WIDTH = 10000;
const float LOG_ODDS_OCCUPIED = 0.65;
const float LOG_ODDS_UNOCCUPIED = 0.35; // 1 - LOG_ODDS_OCCUPIED

const double ZED_FOV = 90.0; //deg

const double CELL_DISTANCE = 0.4; //0.4 meters
const int MAX_FILTER_LENGTH = 7; //7 meters

//(changes)
typedef int8_t CellOdds;   ///< Type used to represent the data in a cell
const int8_t kHitOdds_;
const int8_t kMissOdds_;


//Struct used when reading in odometery data from a text file
//TODO: Write function to read in data from odom.txt and record.txt
//TODO: Write function to read in odom data from LCM synced with image data from LCM
struct Odometry {
    int latitude_deg;
    double latitude_min;
    int longitude_deg;
    double longitude_min;
    double bearing_deg;
    double speed;
};

struct Cell {
    std::size_t colIndex;
    std::size_t rowIndex;
    std::

};

class OccupancyMap {
    private:
        //2D vector that stores the log odds of occupacy for each cell as a float.
        std::vector<std::vector<CellOdds> > occupancyMap;

        //Current cell the rover is located in would be occupancyMap[roverRowIndex][roverColIndex].
        int roverRowIndex, roverColIndex;

        //Current coordinate the rover is located in, with (0.0,0.0) corresponding to the initial gps coordinates of the rover.
        //(0.0,0.0) would be in the center of the cell located in the center of the map (DEFAULT CASE: occupancyMap[4999][4999]).
        float roverXPos, roverYPos;

        //Current Odomerty data of the rover
        Odometry currOdomData;

        double upperFOV, lowerFOV;

    private:
        //Loads obstacle data in the vector representing the 100 cell x 100 cell region surrounding the rover
        //TODO: make the viewing distance a constant (100 x 100)
        void loadObtacles(std::vector<std::vector<int> >& roverFrame /*TODO: add vector of Obstacle Structs as an input*/);

        //Changes the log odd value in the specificed cell
        void updateOccupancyValues (std::size_t& xIndex, std::size_t& yIndex, bool occupied);

        //Calls an OpenCV viewer to view a greyscalled 100 cell x 100 cell region of the occupancy map centered at the rover
        //TODO: Develop this OpenCV viewer function definition
        void viewer();

        //Search surrounding Cell and updates their occupancy
        void searchCells(std::vector<std::vector<int> >& roverFrame, std::queue<CellIndex>& sreach, CellIndex& current);

        void initialise();

        



    public:
        //Contructor with Initial odometry data
        OccupancyMap(const Odometry& initialOdom);

        //Updates the coordinate position and cell position of the rover based on new Odometry data
        void updateRoverPosition(const Odometry& newOdomData);

        //Iterated through all cell in the FOV and updated log odds of occupancy
        void updateOccupancyMap();
        //(changes)
        bool isCellInGrid(int x, int y) const;

        CellOdds logOdds(int x, int y) const;

        //CellOdds& operator()(int x, int y)       { return occupancyMap[x][y]; }

        void setLogOdds(int x, int y, CellOdds logOdds);

        void decreaseCellOdds(int x, int y, OccupancyMap& map);

        void increaseCellOdds(int x, int y, OccupancyMap& map);



};

#endif
