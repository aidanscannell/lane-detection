//
//  controller.cpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 16/11/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#include <opencv2/core.hpp>
#include "opencv2/videoio.hpp"

#include "laneDetectorController.hpp"
#include "vehicleDetectorController.hpp"

#include "IPM.hpp"

#include "laneTracker.hpp"

using namespace cv;
using namespace std;

int main() {

    // Create lane detector controller
    LaneDetectorController lController;
    
    // Create vehicle detector controller
    VehicleDetectorController vController;
    
    // Create instance of base controller
    Controller controller;
    
    // Create windows to displat result
    namedWindow("Lane Detector",CV_WINDOW_AUTOSIZE);
    namedWindow("Vehicle Detector",CV_WINDOW_AUTOSIZE);
    namedWindow("Lane and Vehicle Detector",CV_WINDOW_AUTOSIZE);
    
    // User interface
    cout << "1: to enter haar cascade file path and name" << endl;
    cout << "2: to enter input video file path and name" << endl;
    cout << "3: to enter coordinates for inverse perspective mapping" << endl;
    cout << "4: to run lane detection" << endl;
    cout << "5: to vehicle detection" << endl;
    cout << "6: to lane and vehicle detection" << endl;
    cout << "q: to quit" << endl;
    
    // Initialise user input
    char key = ' ';
    
    // Set default video name
    string video_name = "/Users/Home/Google Drive/Tech/Programming/C++/Projects/cv_autonomous_vehicle/data/videos/ORL_to_Tampa__one_minute.mpeg";
    
    // Set default haar cascade name
    string car_cascade_name = "/Users/Home/Google Drive/Tech/Programming/C++/Projects/vehicle_detection/vehicle_detection/cars.xml";
    
    // Initialise IPM points
    std::vector<cv::Point2f> orgPts;
    std::vector<cv::Point2f> dstPts;
    
    while( (key=getchar()) != 'q' ){
        
        switch (key) {
                
            case '1':
                cout << "Enter haar cascade file path and name" << endl;
                cin >> car_cascade_name;
                break;
                
            case '2':
                cout << "Enter input video file path and name" << endl;
                cin >> video_name;
                break;
                
            case '3':
            {
                // Initialise variables
                float x1, x2, x3, x4, y1, y2, y3, y4;
                Mat frame;
                VideoCapture capIPM(video_name);
                
                // User inputs IPM coordinates
                cout << "Enter coordinates for IPM" << endl;
                cout << "x1: " << endl;
                cin >> x1;
                cout << "y1: " << endl;
                cin >> y1;
                cout << "x2: " << endl;
                cin >> x2;
                cout << "y2: " << endl;
                cin >> y2;
                cout << "x3: " << endl;
                cin >> x3;
                cout << "y3: " << endl;
                cin >> y3;
                cout << "x4: " << endl;
                cin >> x4;
                cout << "y4: " << endl;
                cin >> y4;
                
                orgPts.push_back( Point2f(x1, y1) );
                orgPts.push_back( Point2f(x2, y2) );
                orgPts.push_back( Point2f(x3, y3) );
                orgPts.push_back( Point2f(x4, y4) );
                
                // Create window
                namedWindow("IPM Coordinates",CV_WINDOW_AUTOSIZE);
                
                // Read first frame
                if( !capIPM.isOpened()){
                    cout << "Cannot open the video file" << endl;
                    return -1;
                }
                capIPM.set(CV_CAP_PROP_POS_FRAMES,0);
                
            
                // Create IPM object
                IPM::IPM ipm( Size(capIPM.get(CV_CAP_PROP_FRAME_WIDTH), capIPM.get(CV_CAP_PROP_FRAME_HEIGHT)), Size(capIPM.get(CV_CAP_PROP_FRAME_WIDTH), capIPM.get(CV_CAP_PROP_FRAME_HEIGHT)), orgPts, dstPts );

                // Draw points on frame
                ipm.drawPoints(orgPts, frame);

                // Set the image coordinates for IPM
                lController.initIPM(orgPts);
                
                imshow("IPM Coordinates", frame);
                if(waitKey(27) >= 0) break;
                break;
            
            }
                
                
            case '4':
            {
                // Read video
                VideoCapture cap(video_name);
                
                // Check video was successfully opened
                if (!cap.isOpened()){
                    cout << "Error opening video file!" << endl;
                }
                
                
                // Initialise the Kalman filter
                LaneTracker lTracker, rTracker;
                lTracker.initKalman(0, 0);
                rTracker.initKalman(0, 0);
                
                // Set the kalman filter
                lController.initKalman(lTracker, rTracker);
                
                
                // Loop through video frames
                int counter = 0;
                for(;;) {
                    Mat frame;
                    cap >> frame;
                    
                    // Set the image frame
                    lController.setVideoFrame(frame);
                    
                    // Initialise the IPM points
                    lController.initIPM(orgPts);
                    
                    // Perform lane detection algorithm
                    lController.process();
                    
                    controller.drawResult(frame, lController.getPoints());

                    // Display result
                    imshow("Lane Detector", controller.getLastResult());
                    if(waitKey(30) >= 0) break;
                    counter++;
                }
                break;
            }
                
            case '5':
            {
                // Read video
                VideoCapture cap(video_name);
                
                // Check video was successfully opened
                if (!cap.isOpened()){
                    cout << "Error opening video file!" << endl;
                }
                
                // Set the car cascade
                vController.setCascade(car_cascade_name);
                
                // Loop through video frames
                for(;;) {
                    Mat frame;
                    cap >> frame;
                    
                    // Set the image frame
                    vController.setVideoFrame(frame);
                    
                    // Perform vehicle detection algorithm
                    vController.process();
                    
                    controller.drawResult(frame, vController.getCars());
                    
                    // Display result
                    imshow("Vehicle Detector", controller.getLastResult());
                    if(waitKey(30) >= 0) break;
                }
                break;
            }
                
            case '6':
            {
                // Read video
                VideoCapture cap(video_name);
                
                // Check video was successfully opened
                if (!cap.isOpened()){
                    cout << "Error opening video file!" << endl;
                }
                
                // Initialise the Kalman filter
                LaneTracker lTracker, rTracker;
                lTracker.initKalman(0, 0);
                rTracker.initKalman(0, 0);
                
                // Set the kalman filter
                lController.initKalman(lTracker, rTracker);
                
                // Set the car cascade
                vController.setCascade(car_cascade_name);
                
                // Loop through video frames
                int counter = 0;
                for(;;) {
                    Mat frame;
                    cap >> frame;
                    
                    // Set the image frame
                    lController.setVideoFrame(frame);
                    vController.setVideoFrame(frame);
                    
                    // Initialise the IPM points
                    lController.initIPM(orgPts);
                    
                    // Perform lane detection algorithm
                    lController.process();
                    
                    // Perform vehicle detection algorithm
                    vController.process();
                    
                    controller.drawResult(frame, lController.getPoints(), vController.getCars());
                    
                    // Display result
                    imshow("Lane and Vehicle Detector", controller.getLastResult());
                    if(waitKey(30) >= 0) break;
                    counter++;
                }
                break;
            }
                
            case 'q':
                return 0;
                
        }
    }
}
