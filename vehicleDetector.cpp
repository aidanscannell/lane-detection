//
//  vehicleDetector.cpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 03/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#include "vehicleDetector.hpp"

using namespace cv;
using namespace std;

/********************************************************************************************
 * VEHICLE DETECTION
 ********************************************************************************************
 * This function uses a haar cascade to detect vehicles within the image
 * Output -> The original image with red circles marking the detected vehicles
 * \param frame - the input image
 * \param car_cascade - car cascade for training
 */
vector<Rect> VehicleDetector::process(const Mat &frame, CascadeClassifier car_cascade){
    
    
    
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    
    // Detect cars
    car_cascade.detectMultiScale( frame_gray, cars, 1.1, 2, 0, Size(100, 100) );
    
//            for(int i = 0; i < cars.size(); i++)
//            {
//                cout << "Car" + std::to_string(i) << ": " << cars[i] << " ";
//            }
//            cout  << "\n" << endl;
    
//    for( size_t i = 0; i < cars.size(); i++ ){
//        Mat carsROI = frame_gray( cars[i] );
//        
//        // Draw the car on the image
//        Point center( cars[i].x + cars[i].width/2, cars[i].y + cars[i].height/2 );
//        ellipse( frame, center, Size( cars[i].width/2, cars[i].height/2 ), 0, 0, 360, Scalar( 0, 0, 255 ), 2, 8, 0 );
//    }
    
    return cars;
    
}
