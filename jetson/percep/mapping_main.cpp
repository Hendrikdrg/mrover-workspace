#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;

int main() {

    vector<vector <float> > map(10000, vector<float> (10000, 0.5));

    //We want to view a K x K region of the map in the image, where K << N
    int desiredViewWidthCells = 100;
    int desiredViewStartR = 0;
    int desiredViewStartC = 0;

    //The image we will display, the dimensions can be arbitrary so long as its square
    int desiredViewWidthPx = 800;
    cv::Mat view = cv::Mat(desiredViewWidthPx, desiredViewWidthPx, CV_8UC1);

    int pxWidthPerViewCell = desiredViewWidthPx/desiredViewWidthCells;

    for(int y = 0; y < desiredViewWidthPx - 1; y++) {
        for(int x = 0; x < desiredViewWidthPx - 1; x++) {
            view.at<float>(x, y) = map[desiredViewStartC + x/pxWidthPerViewCell][desiredViewStartR + y/pxWidthPerViewCell];
        }
    }

    imshow("Occupancy Map", view);
}