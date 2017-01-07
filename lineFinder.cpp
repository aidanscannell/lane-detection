//
//  lineFinder.cpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 27/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

// OpenCV header files
#include "opencv2/core.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/shape.hpp"
#include "opencv2/video.hpp"

#include "lineFinder.hpp"

using namespace std;
using namespace cv;


/********************************************************************************************
 * HOUGH TRANSFORM FIND LINES
 ********************************************************************************************
 * This function performs a hough transform in order to detect lines within the input image
 * Output -> vector<[Rho, Theta]> -> Rho is the distance from the coordinate origin, Theta is the line rotation angle in radians
 * \param side - 0 indicates left half of original image, 1 indicates right half of original image
 */
vector<Vec2f> LineFinder::findLines(int side){
    
    lines.clear();
    if (minVote < 1 || lines.size() > 2){
        minVote = 300;
    } else {
        minVote += 10;
    }
    
    // perform hough transform
    while(lines.size() < 5 && minVote > 0){
        HoughLines(imageThres,lines,1,PI/180, minVote, 0, 0);
        minVote -= 10;
    }
    
    return lines;
}

/********************************************************************************************
 * HOUGH TRANSFORM DRAW LINES
 ********************************************************************************************
 * This function draws lines detected using the hough transform
 * Output ->  Mat -> input image with lines overlayed
 * \param image - input image to overlay lines on
 * \param lines - vector containing (rho, theta) pairs for detected lines
 * \param side - 0 indicates left half of original image, 1 indicates right half of original image
 */
Mat LineFinder::drawLines(int side){
    // create output image matrix for hough transform
//    Mat hough(image.size(), CV_8UC1, Scalar(0));
    image.copyTo(hough);
    
    // draw detected lines
    vector<Vec2f>::const_iterator it = lines.begin();
    
    int counter = 0;
    while (it!=lines.end()) {
        
        // only draw the top 10 lines
        if (counter >= 10){
            break;
        }
        
        float rho= (*it)[0];   // distance rho
        float theta= (*it)[1]; // angle theta
        
        Point pt1(rho/cos(theta),0); // intersection - first row
        Point pt2((rho-hough.rows*sin(theta))/cos(theta),hough.rows); // intersection - last row
        
        // plot the lines that could be road
        theta = theta * 180 / 3.141;
        if (side == 0){
            if ( theta >= 0 && theta <= 45 ){
                line( hough, pt1, pt2, Scalar(255), 8);
            }
        } else if (side == 1){
            if ( (theta >= 135 && theta <= 180)){
                line( hough, pt1, pt2, Scalar(255), 8);
            }
        }
        
        ++it;
        ++counter;
        
    }
    return hough;
}


/********************************************************************************************
 * PROBABALISTIC HOUGH TRANSFORM FIND LINES
 ********************************************************************************************
 * This function performs a probabilistic hough transform in order to detect lines within the input image
 * Output ->  vector<[x1,y1,x2,y2]> -> (x1,y1) and (x2,y2) are the ending points of each detected line segment
 * \param side - 0 indicates left half of original image, 1 indicates right half of original image
 */
vector<Vec4i> LineFinder::findLinesP(int side) {
    
    linesP.clear();
    HoughLinesP(imageThres,linesP,deltaRho,deltaTheta,minVote, minLen, maxGap);
    
    return linesP;
}

/********************************************************************************************
 * PROBABALISTIC HOUGH TRANSFORM DRAW LINES
 *******************************************************************************************
 * This function draws lines detected using the probabalistic hough transform
 * Output is the input image with lines overlayed
 * \param image - input image to overlay lines on
 * \param linesP - vector containing (rho, theta) pairs for detected lines
 * \param side - 0 indicates left half of original image, 1 indicates right half of original image
 */
Mat LineFinder::drawLinesP(int side){
    
    // initialise variables
//    Mat result(image.size(), CV_8UC1, Scalar(255));
    image.copyTo(houghP);
    
    // draw detected lines
    vector<Vec4i>::const_iterator it = linesP.begin();
    int counter = 0;
    while (it!=linesP.end()) {
        
        // only draw the top 10 lines
        if (counter >= 10){
            break;
        }
        // create the points
        Point pt1((*it)[0], (*it)[1]);
        Point pt2((*it)[2], (*it)[3]);
        
        // plot the lines that could be road
        float theta = atan2((*it)[1] - (*it)[3], (*it)[0] - (*it)[2]);
        theta = theta * 180 / 3.141;
        if (side == 0){
            if ( theta >= 90 && theta <= 180 ){
                line( houghP, pt1, pt2, Scalar(255), 8);
            }
        } else if (side == 1){
            if ( theta >= -180 && theta <= -90 ){
                line( houghP, pt1, pt2, Scalar(255), 8);
            }
        }
        
        ++counter;
        ++it;
        
    }
    return houghP;
}



//********************************************************************************************
//* SETTERS AND GETTERS
//********************************************************************************************

// Set thresholded image
void LineFinder::setImageThres(cv::Mat imgThres){
    imageThres = imgThres;
}

// Set original image
void LineFinder::setImage(cv::Mat img){
    image = img;
}

// Set min vote
void LineFinder::setMinVote(int min_vote){
    minVote = min_vote;
}

// Set length and gap
void LineFinder::setLenthGap(double len, double gap){
    minLen = len;
    maxGap = gap;
}

// Set res for accumulator
void LineFinder::setRes(double rho, double theta){
    deltaRho = rho;
    deltaTheta = theta;
}

// Get vector of lines from hough transfrom
std::vector<cv::Vec2f> LineFinder::getLines() {
    return lines;
}

// Get vector of lines from probabalistic hough transfrom
std::vector<cv::Vec4i> LineFinder::getLinesP() {
    return linesP;
}

// Get image containing lines from hough transform
cv::Mat LineFinder::getHough() {
    return hough;
}

// Get image containing lines from probabalistic hough transform
cv::Mat LineFinder::getHoughP() {
    return houghP;
}

// Get min vote
int LineFinder::getMinVote(){
    return minVote;
}
