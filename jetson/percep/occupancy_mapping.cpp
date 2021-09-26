#include <iostream>
#include <fstream>
#include <vector>
#include <stack>
#include <cmath>

#include <opencv2/opencv.hpp>
#include "occupancy_mapping.hpp"

OccupancyMap::OccupancyMap() {
    occupancyMap = occupancyMap(DEFAULT_OCCUPANCY_MAP_HEIGHT, std::vector(DEFAULT_OCCUPANCY_MAP_WIDTH, 0.5));
    occupancyMapIteration = occupancyMapIteration(DEFAULT_OCCUPANCY_MAP_HEIGHT, std::vector(DEFAULT_OCCUPANCY_MAP_WIDTH, 0));

    //initialize rover's position in the center
    roverXPos = (DEFAULT_OCCUPANCY_MAP_WIDTH/2) - 1;
    roverYPos = (DEFAULT_OCCUPANCY_MAP_HEIGHT/2) - 1;
}

OccupancyMap::OccupancyMap(int height, int width) {
    //occupancyMap = occupancyMap(height, std::vector(width, 0.5));
    occupancyMap = vector<vector<float> > occupancyMap(height, std::vector(width, 0.5));
    occupancyMapIteration = occupancyMapIteration(height, std::vector(width, 0));

    //initialize rover's position in the center
    roverXPos = (width/2) - 1;
    roverYPos = (height/2) - 1;
}

void OccupancyMap::OccupancyMapViewer() {
    //We want to view a K x K region of the map in the image, where K << N
/*    //We want to view a K x K region of the map in the image, where K << N
    int desiredViewWidthCells = 100;
    int desiredViewStartR = roverXPos;
    int desiredViewStartC = roverYPos;

    //The image we will display, the dimensions can be arbitrary so long as its square
    int desiredViewWidthPx = 800:
    cv::Mat view(cv::Size(desiredViewWidthPx, desiredViewWidthPx));

    int pxWidthPerViewCell = desiredViewWidthPx/desiredViewWidthCells;

    for(int y = 0; y < desiredViewWidthPx; y++) {
        for(int x = 0; x < desiredViewWidthPx; x++) {
            view.at<float>(x, y) = map[desiredViewStartC + x/pxWidthPerViewCell][desiredViewStartR + y/pxWidthPerViewCell];
        }
    }
    cv::imshow("Occupancy Map", view); */
}

void getOdomData() {
    std::ifstream odom;
    odom.open("odom.txt");
    while (!odom.eof) {
        Odometry temp;
        int currImageTime;
        odom >> temp.latitude_deg >> temp.latitude_min >> temp.longitude_deg >> temp.longitude_min;
        odom >> temp.bearing_deg >> temp.speed >> currImageTime;

        odom_data.push_back(temp);
        imageTimeData.push_back(currImageTime);
    }
}

void updateRoverPosition() {
    double changeInDistance = estimateNoneuclid(currentOdometry, previousOdometry);
    double changeInBearing = calcBearing(currentOdometry, previousOdometry);

    while (changeInBearing < 0) {
        changeInBearing += 360.0;
    }
}


