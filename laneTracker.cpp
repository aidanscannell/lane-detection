//
//  laneTracker.cpp
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

#include "laneTracker.hpp"

using namespace cv;
using namespace std;

/********************************************************************************************
 * INITIATE KALMAN FILTER FOR LANE TRACKING
 ********************************************************************************************
 * This function initiates the Kalman filter for lane marker tracking
 * Output -> no output
 * \param rho - line parameter
 * \param theta - line parameter
 */
void LaneTracker::initKalman(float rho, float theta){
    // Create kalman filter with 4 dynamic params, 2 measurement params
    // Measurements are rho & theta of best fit line
    // Dynamic parameters are rho, theta, rho_dot, theta_dot
    laneKalman.init(4, 2, 0);
    
    measurement = Mat_<float>::zeros(2,1);
    measurement.at<float>(0, 0) = rho;
    measurement.at<float>(0, 0) = theta;
    
    laneKalman.statePre.setTo(0);
    laneKalman.statePre.at<float>(0, 0) = rho;
    laneKalman.statePre.at<float>(1, 0) = theta;
    
    laneKalman.statePost.setTo(0);
    laneKalman.statePost.at<float>(0, 0) = rho;
    laneKalman.statePost.at<float>(1, 0) = theta;
    
    setIdentity(laneKalman.transitionMatrix);
    setIdentity(laneKalman.measurementMatrix);
    setIdentity(laneKalman.processNoiseCov, Scalar::all(0.005));
    setIdentity(laneKalman.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(laneKalman.errorCovPost, Scalar::all(0.1));
}

/********************************************************************************************
 * PREDICT PARAMETERS KALMAN FILTER FOR LANE TRACKING
 ********************************************************************************************
 * This function predicts the state
 * Output -> no output
 */
void LaneTracker::predictKalman(){
    Mat prediction = laneKalman.predict();
    predictPt.x = prediction.at<float>(0);
    predictPt.y = prediction.at<float>(1);
    
    laneKalman.statePre.copyTo(laneKalman.statePost);
    laneKalman.errorCovPre.copyTo(laneKalman.errorCovPost);
}

/********************************************************************************************
 * CORRECT KALMAN FILTER FOR LANE TRACKING
 ********************************************************************************************
 * This function corrects the Kalman filter for lane marker tracking
 * Output -> no output
 * \param rho - line parameter
 * \param theta - line parameter
 */
void LaneTracker::correctKalman(float rho, float theta){
    measurement(0) = rho;
    measurement(1) = theta;
    Mat estimated = laneKalman.correct(measurement);
    statePt.x = estimated.at<float>(0);
    statePt.y = estimated.at<float>(1);
}

//********************************************************************************************
//* SETTERS AND GETTERS
//********************************************************************************************
void LaneTracker::setState(cv::Mat state_){
    state = state_;
}

void LaneTracker::setMeasurement(cv::Mat measurement_){
    measurement = measurement_;
}

cv::Point_<float> LaneTracker::getState(){
    return statePt;
}

cv::Point_<float> LaneTracker::getPredicted(){
    return predictPt;
}
