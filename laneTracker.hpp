//
//  laneTracker.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 27/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#ifndef laneTracker_hpp
#define laneTracker_hpp

// OpenCV header files
#include "opencv2/video.hpp"

/*
 * Lane Tracker -> The main class used for tracking lane markers
 */
class LaneTracker {
    
    private:
        
        // Kalman filter for lane marker tracking
        cv::KalmanFilter laneKalman;
        
        // Matrix containing the measured parameters (rho, theta)
        cv::Mat_<float> measurement;
    
        // Matrix containing the state (rho, theta, rho_dot, theta_dot)
        cv::Mat_<float> state;
    
        // Point containing the state
        cv::Point_<float> statePt;
    
        // Point containing the predicted
        cv::Point_<float> predictPt;
        
    public:
    
        /********************************************************************************************
         * INITIATE KALMAN FILTER FOR LANE TRACKING
         ********************************************************************************************
         * This function initiates the Kalman filter for lane marker tracking
         * Output -> no output
         * \param rho - line parameter
         * \param theta - line parameter
         */
        void initKalman(float rho, float theta);
    
        /********************************************************************************************
         * PREDICT PARAMETERS KALMAN FILTER FOR LANE TRACKING
         ********************************************************************************************
         * This function predicts the state
         * Output -> no output
         */
        void predictKalman();
    
        /********************************************************************************************
         * CORRECT KALMAN FILTER FOR LANE TRACKING
         ********************************************************************************************
         * This function corrects the Kalman filter for lane marker tracking
         * Output -> no output
         * \param rho - line parameter
         * \param theta - line parameter
         */
        void correctKalman(float rho, float theta);
        
        //********************************************************************************************
        //* SETTERS AND GETTERS
        //********************************************************************************************
        void setState(cv::Mat state_);
    
        void setMeasurement(cv::Mat measurement_);
    
        cv::Point_<float> getState();
    
        cv::Point_<float> getPredicted();
    
};

#endif /* laneTracker_hpp */
