//
//  vehicleDetectorController.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 03/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#ifndef vehicleDetectorController_h
#define vehicleDetectorController_h

#include "vehicleDetector.hpp"

#include "controller.hpp"

class VehicleDetectorController: public Controller {
    
    private:
        
        // Algorithm class
        VehicleDetector *vdetect;
    
        // Haar cascade
        cv::CascadeClassifier car_cascade;
    
        // Vector containing detected cars
        std::vector<cv::Rect> cars;
    
    public:
        
        VehicleDetectorController (){ // private constructor
            
            // Setting up the application
            vdetect = new VehicleDetector();
        }
    
        // Load the haar cascade
        bool setCascade(cv::String car_cascade_name){
            
            if( !car_cascade.load( car_cascade_name ) ){
                return false;
            } else {
                return true;
            }
        }
    
        // Perform processing
        void process() {
            
            cars = vdetect->process(image, car_cascade);
        }
    
        // Get the vector of detected cars
        std::vector<cv::Rect> getCars(){
            return cars;
        }
    
        // Delete all processor objects created by controller
        ~VehicleDetectorController() {
            
            delete vdetect;
        }
};

#endif /* vehicleDetectorController_h */
