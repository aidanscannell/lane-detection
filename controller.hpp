//
//  controller.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 23/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#ifndef controller_hpp
#define controller_hpp

/* 
 * Base Controller -> This is the base controller class
 */
class Controller {
    
    protected:
    
        // Image to be processed
        cv::Mat image;
        
        // Resulting image
        cv::Mat result;
    
    public:
    
        // Read input frame
        bool setVideoFrame(cv::Mat frame){
            
            image = frame;
            return true;
        }
        
        // Return input image
        const cv::Mat getInputImage() const {
            
            return image;
        }
        
        // Perform processing
        virtual void process() {}
    
        //Draws the lane markers on the original image
        void drawResult(cv::Mat frame, std::vector<float> points){
            frame.copyTo(result);

            // plot the lines that could be road
            cv::line( result, cv::Point (points[0],points[1]), cv::Point (points[2],points[3]), cv::Scalar(255), 8);
        
            // plot the lines that could be road
            cv::line( result, cv::Point (points[4],points[5]), cv::Point (points[6],points[7]), cv::Scalar(255), 8);
            
        }
    
        // Draw detected vehicles on input image
        void drawResult(cv::Mat frame, std::vector<cv::Rect> cars){
            frame.copyTo(result);
            
            for( size_t i = 0; i < cars.size(); i++ ){
                
                // Draw the car on the image
                cv::Point center( cars[i].x + cars[i].width/2, cars[i].y + cars[i].height/2 );
                ellipse( result, center, cv::Size( cars[i].width/2, cars[i].height/2 ), 0, 0, 360, cv::Scalar( 0, 0, 255 ), 2, 8, 0 );
            }
            
        }
    
        // Draw detected vehicles and lane markers on input image
        void drawResult(cv::Mat frame, std::vector<float> points, std::vector<cv::Rect> cars){
            frame.copyTo(result);
            
            // plot the lines that could be road
            cv::line( result, cv::Point (points[0],points[1]), cv::Point (points[2],points[3]), cv::Scalar(255), 8);
            
            // plot the lines that could be road
            cv::line( result, cv::Point (points[4],points[5]), cv::Point (points[6],points[7]), cv::Scalar(255), 8);
            
            for( size_t i = 0; i < cars.size(); i++ ){
                
                // Draw the car on the image
                cv::Point center( cars[i].x + cars[i].width/2, cars[i].y + cars[i].height/2 );
                ellipse( result, center, cv::Size( cars[i].width/2, cars[i].height/2 ), 0, 0, 360, cv::Scalar( 0, 0, 255 ), 2, 8, 0 );
            }
            
        }

    
        // Get the result
        const cv::Mat getLastResult() const {
            
            return result;
        }
    
};

#endif /* controller_hpp */
