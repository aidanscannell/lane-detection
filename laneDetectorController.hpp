//
//  lineFinderController.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 16/11/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#ifndef laneDetectorController_hpp
#define laneDetectorController_hpp

#include "laneDetector.hpp"

#include "laneTracker.hpp"

#include "controller.hpp"

class LaneDetectorController: public Controller {
    
    private:
    
        // Algorithm class
        LaneDetector *ldetect;
    
        // Vector containing lane marker points
        std::vector<float> resultPts;
    
    public:
        
        LaneDetectorController (){ // private constructor
            
            // Setting up the application
            ldetect = new LaneDetector();
        }
    
        // Initialise the IPM points
        void initIPM(std::vector<cv::Point2f> orgPts){
            
            // Set default destination points
            std::vector<cv::Point2f> dstPts;
            dstPts.push_back( cv::Point2f(0, image.rows) );
            dstPts.push_back( cv::Point2f(image.cols, image.rows) );
            dstPts.push_back( cv::Point2f(image.cols, 0) );
            dstPts.push_back( cv::Point2f(0, 0) );
            
            // Set defualt image coordinates for IPM if not already set
            if (orgPts.empty()){
                orgPts.push_back( cv::Point2f(0, image.rows) );
                orgPts.push_back( cv::Point2f(image.cols, image.rows) );
                orgPts.push_back( cv::Point2f(image.cols/2+150, 700) );
                orgPts.push_back( cv::Point2f(image.cols/2-300, 700) );
            }
            
            ldetect->setDstPts(dstPts);
            ldetect->setOrgPts(orgPts);
        }
    
        // Perform processing
        void process() {
            
            resultPts = ldetect->process(image);
        }
    
        // Initialise the Kalman filters
        void initKalman(LaneTracker lTrack, LaneTracker rTrack){
            
            ldetect->initKalman(lTrack, rTrack);
        }
    
        // Get the points for the detected lane markers
        std::vector<float> getPoints(){
            return resultPts;
        }
    
        // Delete all processor objects created by controller
        ~LaneDetectorController() {
            
            delete ldetect;
        }
    
};

#endif /* laneDetectorController_hpp */
