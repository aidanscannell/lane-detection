//
//  vehicleDetector.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 03/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#ifndef vehicleDetector_hpp
#define vehicleDetector_hpp

#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <stdio.h>

/*
 * Vehicle Detector -> The main class for vehicle detection
 */
class VehicleDetector {
    
    private:
        
        // Input image
        cv::Mat image;
        
        // Output image
        cv::Mat result;
    
        // Haar cascade
        cv::CascadeClassifier car_cascade;
    
        // Vector containing the detected cars
        std::vector<cv::Rect> cars;
    
        // Image containing the gray scale image
        cv::Mat frame_gray;
    
    public:
    
        /*******************************************************************************************
         * VEHICLE DETECTOR
         *******************************************************************************************
         * This is the main function that performs the vehicle detection
         * Output is the input image with the detected vehicle marked by red circlew
         * \param image -> the input image (selected frame)
         * \param car_cascade -> the car cascade for the haar detector
         */
        std::vector<cv::Rect> process(const cv::Mat &image, cv::CascadeClassifier car_cascade);
};

#endif /* vehicleDetector_hpp */
