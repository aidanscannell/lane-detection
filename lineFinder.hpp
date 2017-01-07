//
//  lineFinder.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 27/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#ifndef lineFinder_hpp
#define lineFinder_hpp

# define PI 3.14159265358979323846  /* pi */

/*
 * Lane Detector -> The main class used for detecting lane markers
 */
class LineFinder{
    
private:
    
    // Original image
    cv::Mat image;
    
    // Thresholded image
    cv::Mat imageThres;

    // Vector of (rho,theta) for detected lines from hough transform
    std::vector<cv::Vec2f> lines;
    
    // Vector of end points of detected lines from probabalistic hough transform
    std::vector<cv::Vec4i> linesP;
    
    // Image containing lines from hough transform
    cv::Mat hough;
    
    // Image containing lines from probabalistic hough transform
    cv::Mat houghP;

    // Res parameters for accumulation
    double deltaRho;
    double deltaTheta;
    
    // Minimum length for a line
    double minLen;
    
    // Min votes required for a line to be considered
    int minVote;
    
    // Max gap along line
    double maxGap;
    
public:
    
    // Default parameter initialization
    LineFinder() : deltaRho(2.5), deltaTheta(PI/180), minVote(80), minLen(200), maxGap(30) {}
    
    /********************************************************************************************
     * HOUGH TRANSFORM FIND LINES
     ********************************************************************************************
     * This function performs a hough transform in order to detect lines within the input image
     * Output -> vector<[Rho, Theta]> -> Rho is the distance from the coordinate origin, Theta is the line rotation angle in radians
     * \param image - is the image containing edges (either canny or adaptive threshold)
     * \param side - 0 indicates left half of original image, 1 indicates right half of original image
     */
    std::vector<cv::Vec2f> findLines(int side);
    
    /********************************************************************************************
     * HOUGH TRANSFORM DRAW LINES
     ********************************************************************************************
     * This function draws lines detected using the hough transform
     * Output ->  Mat -> input image with lines overlayed
     * \param image - input image to overlay lines on
     * \param lines - vector containing (rho, theta) pairs for detected lines
     * \param side - 0 indicates left half of original image, 1 indicates right half of original image
     */
    cv::Mat drawLines(int side);
    
    /********************************************************************************************
     * PROBABALISTIC HOUGH TRANSFORM FIND LINES
     ********************************************************************************************
     * This function performs a probabilistic hough transform in order to detect lines within the input image
     * Output ->  vector<[x1,y1,x2,y2]> -> (x1,y1) and (x2,y2) are the ending points of each detected line segment
     * \param image - is the image containing edges (either canny or adaptive threshold)
     * \param side - 0 indicates left half of original image, 1 indicates right half of original image
     */
    std::vector<cv::Vec4i> findLinesP(int side);
    
    /********************************************************************************************
     * PROBABALISTIC HOUGH TRANSFORM DRAW LINES
     *******************************************************************************************
     * This function draws lines detected using the probabalistic hough transform
     * Output is the input image with lines overlayed
     * \param image - input image to overlay lines on
     * \param linesP - vector containing (rho, theta) pairs for detected lines
     * \param side - 0 indicates left half of original image, 1 indicates right half of original image
     */
    cv::Mat drawLinesP(int side);

    //********************************************************************************************
    //* SETTERS AND GETTERS
    //********************************************************************************************
    
    // Set original image
    void setImage(cv::Mat img);
    
    // Set thresholded image
    void setImageThres(cv::Mat imgThres);
    
    // Set min vote
    void setMinVote(int min_vote);
    
    // Set length and gap
    void setLenthGap(double len, double gap);
    
    // Set res for accumulator
    void setRes(double rho, double theta);
    
    // Get vector of lines from hough transfrom
    std::vector<cv::Vec2f> getLines();
    
    // Get vector of lines from probabalistic hough transfrom
    std::vector<cv::Vec4i> getLinesP();
    
    // Get image containing lines from hough transform
    cv::Mat getHough();
    
    // Get image containing lines from probabalistic hough transform
    cv::Mat getHoughP();
    
    // Get min vote
    int getMinVote();
};


#endif /* lineFinder_hpp */
