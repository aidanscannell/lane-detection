//
//  laneDetector.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 15/11/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#ifndef laneDetector_hpp
#define laneDetector_hpp

# define PI 3.14159265358979323846  /* pi */

#include "laneTracker.hpp"
#include "IPM.hpp"
/*
 * Lane Detector -> The main class used for detecting lane markers
 */
class LaneDetector{
    
    private:
    
        // Original image
        cv::Mat imageOrg;
    
        // IPM image
        cv::Mat image;
    
//        // 1 channel grayscale image
//        cv::Mat imgG;
    
        // IPM -> Selected 4 points on input image
        std::vector<cv::Point2f> orgPts;
    
        // IPM -> Selected 4 points on transformed image
        std::vector<cv::Point2f> dstPts;
    
        // Adaptive threshold parameters
        int blockSizeAt; // blocksize
        int cAt; // constant subtracted from the mean or weighted mean
    
        // Black image containing white lines from hough transform
        cv::Mat hough;
    
        // Number of sample points
        int nSample;
    
        // Vector for least squares line
        cv::Vec4f lsLine;
    
        // Image containing best fit line
        cv::Mat imgBestFit;
    
        // Vector of rho, theta for best fit line
        cv::Vec2f lines;
    
        // Lane tracker object
        LaneTracker lTracker, rTracker;
    
        // Image containing the result
        cv::Mat result;
    
    public:
    
        // Default parameter initialization
        LaneDetector() : blockSizeAt(15), cAt(-5), nSample(30){}
    
        /********************************************************************************************
         * DETECR LANES
         ********************************************************************************************
         * This function detects possible lane markers within the image
         * Ouput is a black image containing the 10 most probable lines
         * \param image -  input image
         * \param side -  left or right
         */
        void detectLanes(cv::Mat &image, int side);
    
        /*******************************************************************************************
         * LANE DETECTOR
         *******************************************************************************************
         * This is the main function that performs the lane detection
         * Output is the input image with the detected lanes overlayed
         * \param image -> the input image (selected frame)
         */
        std::vector<float> process(const cv::Mat &image);
    
        /********************************************************************************************
         * FIND RHO, THETA FOR BEST FOR LINE
         ********************************************************************************************
         * This function calculates the vector<[Rho, Theta]> for the best fit lien
         * Output -> vector<[Rho, Theta]> -> Rho is the distance from the coordinate origin, Theta is the line rotation angle in radians
         * \param imageL - is the left image containing  the best fit line
         * \param imageR - is the right image containing  the best fit line
         */
        void calcLineParams(int side);

        /********************************************************************************************
         * DRAW DASHED LINE
         ********************************************************************************************
         * This function draws a dashed line of "+" between the specified points
         * Output is the input image with the a dashed line between the specified points
         * \param image - is the input image
         * \param startPoint - start of line point
         * \param endPoint - end of line point
         * \param n - number of dashes
         * \param colour - colour of the line
         */
        void drawDashedLine(cv::Mat image, cv::Point startPoint, cv::Point endPoint, int n, cv::Scalar colour);
    
        /********************************************************************************************
         * SAMPLE LINES
         ********************************************************************************************
         * This function samples along the length of the lines and performs least squares regression for finding best fit line
         * Output is the best fit line (overlayed on original image if overlayFlag=1)
         * \param sampleImg - the image containing the lines output form the hough transform
         * \param orgImg - the original image
         * \param nSample - the number of sample locations along the height of the image
         * \param overlayFlag - 1 results in best fit line being overlayed on original image
         */
        void sampleLine(int overlayFlag);
    
        /********************************************************************************************
         * INVERSE PERSECTIVE MAPPING
         ********************************************************************************************
         * This function maps the camera view to a birds eye view
         * Ouput is the birds eye view image of the input
         * \param image -  is the input image
         */
        cv::Mat ipmImg(cv::Mat image);
    
        /********************************************************************************************
         * INITIATE KALMAN FILTER
         ********************************************************************************************
         * This function draws the lane marker on the original image
         * Ouput is the original image with the line overlayed on it
         * \param lTrack - lane tracker objecy for left lane
         * \param rTrack - lane tracker objecy for right lane
         */
        void initKalman(LaneTracker lTrack, LaneTracker rTrack);
    
        /********************************************************************************************
         * DRAW LANE MARKER
         ********************************************************************************************
         * This function draws the lane marker on the original image
         * Ouput is the original image with the line overlayed on it
         * \param rho -  line parameter
         * \param theta -  line parameter
         */
        void drawResult(float rho, float theta, IPM ipm, int side);
    
        std::vector<float> calcResult(float rho, float theta, IPM ipm, int side);
    
    
        //********************************************************************************************
        //* SETTERS AND GETTERS
        //********************************************************************************************

        // Set input image
        void setImageOrg(cv::Mat image_);
    
        // Set input image
        void setImageIPM(cv::Mat image_);
    
        // Set original points for IPM
        void setOrgPts(std::vector<cv::Point2f> org_Pts);
    
        // Set transformed points for IPM
        void setDstPts(std::vector<cv::Point2f> dst_Pts);
    
        // Set transformed points for IPM
        void setLines(cv::Vec2f lines_);
    
        // Set number of sample points
        void setSampleN(int sample);
    
        // Get original image
        cv::Mat getImgOrg();
        
        // Get IPM image
        cv::Mat getImgIPM();
    
        // Get hough image
        cv::Mat getHough();
    
        // Get image with detected lane markers
        cv::Mat getImgBestFit();
    
        // Get the vector of rho, theta for best fit line
        cv::Vec2f getLines();
    
        // Get result with best fit line overlayed on original image
        cv::Mat getResult();
    
};


#endif /* laneDetector_hpp */
