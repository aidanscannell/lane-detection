////
////  laneDetector.cpp
////  cv_autonomous_vehicle
////
////  Created by AidanScannell on 16/11/2016.
////  Copyright © 2016 AidanScannell. All rights reserved.
////
//

// OpenCV header files
#include "opencv2/core.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/shape.hpp"
#include "opencv2/video.hpp"

// Inverse perspective mapping header file
#include "IPM.hpp"

// Line finder header file
#include "lineFinder.hpp"

// Lane detector header file
#include "laneDetector.hpp"

// Lane tracker header file
#include "laneTracker.hpp"

#define dtof(d) ((float)d)

using namespace cv;
using namespace std;
    
/*******************************************************************************************
 * LANE DETECTOR
 *******************************************************************************************
 * This is the main function that performs the lane marker detection
 * Output is the input image with the detected lanes overlayed
 * \param image -> the input image (selected frame)
 */
vector<float> LaneDetector::process(const Mat &image){
    
    //******************************************************************************************
    // Pre-Processing
    //******************************************************************************************
    // Set region of interest
    //    Rect ROI = Rect(image.cols/7,image.rows/1.33,image.cols-image.cols/7,image.rows-image.rows/1.33);
    //    at imgROI = image(ROI);
    Mat imgROI;
    image.copyTo(imgROI);
    
    // Convert to grayscale
    Mat imgG(imgROI.size(),CV_8UC1, Scalar(0)); // 1 channel grayscale image
    if(image.channels() == 3){
        cvtColor(imgROI, imgG, CV_BGR2GRAY);
    } else {
        imgROI.copyTo(imgG);
    }
    
    // Inverse perspective mapping
    IPM::IPM ipm( Size(imgG.cols, imgG.rows), Size(imgG.cols, imgG.rows), orgPts, dstPts ); // IPM object
    Mat imgIpm; // ipm image
    ipm.applyHomography( imgG, imgIpm );
    
    // Create lane detector instances for left and right lanes
    LaneDetector lDetect, rDetect;
    
    // Split the original image into two halves
    lDetect.setImageOrg(imgROI(Rect (0,0,imgROI.cols/2,imgROI.rows)));
    rDetect.setImageOrg(imgROI(Rect (imgROI.cols/2,0,imgROI.cols-imgROI.cols/2,imgROI.rows)));
    
    // Split the IPM image into two halves
    lDetect.setImageIPM(imgIpm(Rect (0,0,image.cols/2,image.rows)));
    rDetect.setImageIPM(imgIpm(Rect (image.cols/2,0,image.cols-image.cols/2,image.rows)));
    
    //******************************************************************************************
    // Detect the lanes
    //******************************************************************************************
    Mat imgL, imgR;
    lDetect.getImgIPM().copyTo(imgL);
    rDetect.getImgIPM().copyTo(imgR);
    lDetect.detectLanes(imgL, 0);
    rDetect.detectLanes(imgR, 1);
    
    //******************************************************************************************
    // Sample along image to find most probable center of lane marker
    //******************************************************************************************
    lDetect.sampleLine(0); // int -> overlay flag
    rDetect.sampleLine(0); // int -> overlay flag
    
    //******************************************************************************************
    // Calculate rho, theta for best fit line
    //******************************************************************************************
    lDetect.calcLineParams(0);
    rDetect.calcLineParams(1);

    //******************************************************************************************
    // Kalman filter for lane tracking
    //******************************************************************************************
    // Predict parameters
    lTracker.predictKalman();
    rTracker.predictKalman();
    
    // Correct the kalmen filter if a line was found
    if ( rDetect.getLines()[0] != 0 && rDetect.getLines()[1] != 0){
        rTracker.correctKalman(rDetect.getLines()[0], rDetect.getLines()[1]);
    }
    if ( lDetect.getLines()[0] != 0 && lDetect.getLines()[1] != 0){
        lTracker.correctKalman(lDetect.getLines()[0], lDetect.getLines()[1]);
    }
    
//    //******************************************************************************************
//    // Drawn lane markers on the original image
//    //******************************************************************************************
//    // If no line found use Kalman prediction
//    if ( lDetect.getLines()[0] == 0 && lDetect.getLines()[1] == 0){
//        lDetect.drawResult(lTracker.getPredicted().x, lTracker.getPredicted().y,ipm, 0);
//    } else {
//        lDetect.drawResult(lDetect.getLines()[0], lDetect.getLines()[1], ipm, 0);
//    }
//    if ( rDetect.getLines()[0] == 0 && rDetect.getLines()[1] == 0){
//        rDetect.drawResult(rTracker.getPredicted().x, rTracker.getPredicted().y, ipm, 1);
//    } else {
//        rDetect.drawResult(rDetect.getLines()[0], rDetect.getLines()[1], ipm, 1);
//    }
    
    //******************************************************************************************
    // Calcualte lane markers on the original image
    //******************************************************************************************
    // If no line found use Kalman prediction
    vector<float> outputPts;
    vector<float> outputPts2;
    if ( lDetect.getLines()[0] == 0 && lDetect.getLines()[1] == 0){
        outputPts2 = lDetect.calcResult(lTracker.getPredicted().x, lTracker.getPredicted().y,ipm, 0);
    } else {
        outputPts2 = lDetect.calcResult(lDetect.getLines()[0], lDetect.getLines()[1], ipm, 0);
    }
    outputPts.insert(std::end(outputPts), std::begin(outputPts2), std::end(outputPts2));
    if ( rDetect.getLines()[0] == 0 && rDetect.getLines()[1] == 0){
        outputPts2 = rDetect.calcResult(rTracker.getPredicted().x, rTracker.getPredicted().y, ipm, 1);
    } else {
        outputPts2 = rDetect.calcResult(rDetect.getLines()[0], rDetect.getLines()[1], ipm, 1);
    }
    outputPts.insert(std::end(outputPts), std::begin(outputPts2), std::end(outputPts2));
    
//    cout << "pts2 1:" << outputPts[0] << endl;
//    cout << "pts2 2: " << outputPts[1] << endl;
//    cout << "pts2 3: " << outputPts[2] << endl;
//    cout << "pts2 4: " << outputPts[3] << endl;
    
//    // Display the line that was found and the line predicted by the kalman filter
//    cout << "actual line: " << rDetect.getLines()[0] << " " << rDetect.getLines()[1] << endl;
//    cout << "kalman prediction: " << rTracker.getPredicted().x << " " << rTracker.getPredicted().y << endl;

    //******************************************************************************************
    // Merge the two halves together
    //******************************************************************************************
    Mat output(imgIpm.size(),CV_8UC3,Scalar(0,0,0)); // 3 channel output image
    hconcat(lDetect.getResult(), rDetect.getResult(), output);

    return outputPts;
}

/********************************************************************************************
 * DRAW LANE MARKER
 ********************************************************************************************
 * This function draws the lane marker on the original image
 * Ouput is the original image with the line overlayed on it
 * \param rho -  line parameter
 * \param theta -  line parameter
 */
void LaneDetector::drawResult(float rho, float theta, IPM ipm, int side){
    imageOrg.copyTo(result);
    
    if (side == 0){
        Point pt1R(rho/cos(theta),0); // intersection - first row
        Point pt2R((rho-result.rows*sin(theta))/cos(theta),result.rows); // intersection - last row
        
        pt1R = ipm.applyHomographyInv(Point2f(pt1R.x, pt1R.y));
        pt2R = ipm.applyHomographyInv(Point2f(pt2R.x, pt2R.y));

        // plot the lines that could be road
        line( result, pt1R, pt2R, Scalar(255), 8);
    }
    if (side == 1){
        Point pt1R(rho/cos(theta)+result.cols,0); // intersection - first row
        Point pt2R((rho-result.rows*sin(theta))/cos(theta)+result.cols,result.rows); // intersection - last row
        
        pt1R = ipm.applyHomographyInv(Point2f(pt1R.x, pt1R.y));
        pt2R = ipm.applyHomographyInv(Point2f(pt2R.x, pt2R.y));
        
        Mat output(Size (1920,1080),CV_8UC3,Scalar(0,0,0)); // 3 channel output image
        hconcat(result, result, output);
        
        // plot the lines that could be road
        line( output, pt1R, pt2R, Scalar(255), 8);
        
        result = output(Rect (output.cols/2,0,output.cols-output.cols/2,output.rows)); // right half image
    }
}

/********************************************************************************************
 * CALC LANE MARKER
 ********************************************************************************************
 * This function draws the lane marker on the original image
 * Ouput is the original image with the line overlayed on it
 * \param rho -  line parameter
 * \param theta -  line parameter
 */
vector<float> LaneDetector::calcResult(float rho, float theta, IPM ipm, int side){
    imageOrg.copyTo(result);
    
    if (side == 0){
        Point pt1R(rho/cos(theta),0); // intersection - first row
        Point pt2R((rho-result.rows*sin(theta))/cos(theta),result.rows); // intersection - last row
        
        pt1R = ipm.applyHomographyInv(Point2f(pt1R.x, pt1R.y));
        pt2R = ipm.applyHomographyInv(Point2f(pt2R.x, pt2R.y));
        
        vector<float> outputPts;
        outputPts.push_back(pt1R.x);
        outputPts.push_back(pt1R.y);
        outputPts.push_back(pt2R.x);
        outputPts.push_back(pt2R.y);
        return outputPts;
    } else {
        Point pt1R(rho/cos(theta)+result.cols,0); // intersection - first row
        Point pt2R((rho-result.rows*sin(theta))/cos(theta)+result.cols,result.rows); // intersection - last row
        
        pt1R = ipm.applyHomographyInv(Point2f(pt1R.x, pt1R.y));
        pt2R = ipm.applyHomographyInv(Point2f(pt2R.x, pt2R.y));
        
        Mat output(Size (1920,1080),CV_8UC3,Scalar(0,0,0)); // 3 channel output image
        hconcat(result, result, output);
        
        // plot the lines that could be road
        line( output, pt1R, pt2R, Scalar(255), 8);
        
        result = output(Rect (output.cols/2,0,output.cols-output.cols/2,output.rows)); // right half image
        
        vector<float> outputPts;
        outputPts.push_back(pt1R.x);
        outputPts.push_back(pt1R.y);
        outputPts.push_back(pt2R.x);
        outputPts.push_back(pt2R.y);
        return outputPts;
    }
}


/********************************************************************************************
 * DETECT LANES
 ********************************************************************************************
 * This function detects possible lane markers within the image
 * Ouput is a black image containing the 10 most probable lines
 * \param image -  input image
 * \param side -  left or right
 */
void LaneDetector::detectLanes(Mat &image, int side){
    
    // Apply adaptive threshold
    Mat imgAt(image.size(),CV_8UC1, Scalar(0)); // 1 channel left adaptive threshold image
    adaptiveThreshold(image, imgAt, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,blockSizeAt,cAt);

    
    //******************************************************************************************
    // Hough Transform to find lanes
    //******************************************************************************************
    LineFinder finder; // create instance of line finder class
    
    // Set parameters
    finder.setLenthGap(200,30);
    
    // Set images
    finder.setImage(image);
    finder.setImageThres(imgAt);
    
    // Perform hough transform to find lines
    finder.findLines(side); // vector of lines generated from left lane hough transform
    
    // Draw hough lines
    finder.drawLines(side); // detected left lane lines overlayed on original image
    
    
    //******************************************************************************************
    // Probabalistic Hough Transform to find lanes
    //******************************************************************************************
    // Perform probabalistic hough transform to find lines
    finder.findLinesP(side);
    
    // Draw probabilistic hough lines
    finder.drawLinesP(side); // detected left lane lines overlayed on original image
    
    
    //******************************************************************************************
    // Combine results and generate possible lines
    //******************************************************************************************
    // "bitwise_and" of probabilistic and normal hough transforms
    Mat imgBit(image.size(),CV_8UC1,Scalar(0)); // 1 channel image for resulting lane marker lines
    bitwise_and(finder.getHoughP(),finder.getHough(),imgBit);
    
    // Invert resulting image from bitwise operation and perform adaptive thresholding
    Mat imgInv(image.size(),CV_8UC1,Scalar(0)); // 1 channel image for inverted left lane
    Mat imgBitAt(image.size(),CV_8UC1,Scalar(0)); // 1 channel image for thresholded left lane
    threshold(imgBit,imgInv,150,255,THRESH_BINARY_INV);
    adaptiveThreshold(imgInv, imgBitAt, 255, ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,blockSizeAt,cAt);
    
    // Create instances of line finder class
    LineFinder finderB;
    
    // Set parameters for hough transform
    finderB.setLenthGap(60,10); // set length and gap
    finderB.setMinVote(4); // set the minimum vote
    
    // Set images
    Mat sample(image.size(),CV_8UC1,Scalar(0)); // 1 channel black image
    finderB.setImage(sample);
    finderB.setImageThres(imgBitAt);
    
    // Perform Hough Transform to find lines
    finderB.findLines(side);
    
    // Draw lines on black image for sampling
    finderB.drawLines(side);
    
    hough = finderB.getHough();
}


/********************************************************************************************
 * FIND RHO, THETA FOR BEST FOR LINE
 ********************************************************************************************
 * This function calculates the vector<[Rho, Theta]> for the best fit lien
 * Output -> vector<[Rho, Theta]> -> Rho is the distance from the coordinate origin, Theta is the line rotation angle in radians
 * \param imageL - is the left image containing  the best fit line
 * \param imageR - is the right image containing  the best fit line
 */
void LaneDetector::calcLineParams(int side){
    
    // Create instance of line finder class
    LineFinder finder;

    // Set the image input to the black black image containing the line of best fir
    finder.setImageThres(imgBestFit);

    finder.findLines(side);

    if (!finder.getLines().empty()){
//        detect.setLines(finder.getLines().front());
        lines = finder.getLines().front();
//        cout << "lines: " << lines << endl;
    }
}

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
void LaneDetector::drawDashedLine(Mat image, Point startPoint, Point endPoint, int n, Scalar colour){
    
    // Pixels along the line
    LineIterator it(image, startPoint, endPoint, 8);
    
    // Iterate through the line
    for(int i = 0; i < it.count; i++, ++it)
    {
        
        if (i % n == 0){
            putText(image, "+", it.pos(), 0, 1, colour);
        }
    }
}

/********************************************************************************************
 * SAMPLE LINES
 ********************************************************************************************
 * This function samples along the length of the lines and performs least squares regression for finding best fit line
 * Output is the best fit line (overlayed on original image if overlatFlag=1)
 * \param sampleImg - the image containing the lines output form the hough transform
 * \param orgImg - the original image
 * \param nSample - the number of sample locations along the height of the image
 * \param overlayFlag - 1 results in best fit line being overlayed on original image
 */
void LaneDetector::sampleLine(int overlayFlag){
    vector<Point> pts; // initialise vector of points
    pts.clear();
    
    // Set image with best fit line to all zeros
    Mat m(hough.size(),CV_8UC1,Scalar(0));
    imgBestFit = m;
    
    // Overlay line on original image if flag set to 1
    if (overlayFlag == 1){
        image.copyTo(imgBestFit);
    }
    
    for (int i = 0; i < nSample; i++){ // iterate through rows (y)
        
        // calculate row for given number of sample points
        int row = (i+1) * (hough.rows / nSample);
        
        for (int col = 0; col < hough.cols; col++){ // iterate through columns (x)
            
            // if line present in pixel store location in pts vector
            Scalar colour = hough.at<uchar>(Point(row, col));
            if(colour.val[0]==255){
                
                pts.push_back(Point(row,col));
                
                // draw point on image
//                putText(sampledR, "+", Point(col,row), 0, 1, Scalar(255));
            }
        }
    }
    
    if (!pts.empty()){
        
        // fit a line through the points using least squares regression
        fitLine(pts, lsLine, CV_DIST_HUBER, 0, 0.01, 0.01);
        
        // calculate the line start and end points
        Point startPoint, endPoint;
        startPoint.x = lsLine[2]- imgBestFit.cols*lsLine[0];// x0
        startPoint.y = lsLine[3] - imgBestFit.cols*lsLine[1];// y0
        endPoint.x = lsLine[2]+ imgBestFit.cols*lsLine[0];//x[1]
        endPoint.y = lsLine[3] + imgBestFit.cols*lsLine[1];//y[1]
        
        // draw least squares line on image
        line(imgBestFit, startPoint, endPoint, Scalar(255), 8);
    }
}

/********************************************************************************************
 * INITIATE KALMAN FILTER
 ********************************************************************************************
 * This function draws the lane marker on the original image
 * Ouput is the original image with the line overlayed on it
 * \param lTrack - lane tracker objecy for left lane
 * \param rTrack - lane tracker objecy for right lane
 */
void LaneDetector::initKalman(LaneTracker lTrack, LaneTracker rTrack){
    
    // Create instances of lane tracker class
    lTracker = lTrack;
    rTracker = rTrack;
}

//// gaussian blur
//Mat gaussianBlur (Mat &input, int MAX_KERNEL_LENGTH){
//
//    Mat output;
//    for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 ){
//        GaussianBlur( input, output, Size( i, i ), 0, 0 );
//    }
//    return output;
//}



//********************************************************************************************
//* SETTERS AND GETTERS
//********************************************************************************************

// Set original image
void LaneDetector::setImageOrg(cv::Mat image_){
    imageOrg = image_;
}
// Set input IPM image
void LaneDetector::setImageIPM(cv::Mat image_){
    image = image_;
}

// Set number of sample points
void LaneDetector::setSampleN(int sample){
    nSample = sample;
}

// Set original points for IPM
void LaneDetector::setOrgPts(std::vector<cv::Point2f> org_Pts){
    orgPts = org_Pts;
}

// Set transformed points for IPM
void LaneDetector::setDstPts(std::vector<cv::Point2f> dst_Pts){
    dstPts = dst_Pts;
}

// Set transformed points for IPM
void LaneDetector::setLines(cv::Vec2f lines_){
    lines = lines_;
}

// Get image half
cv::Mat LaneDetector::getImgBestFit(){
    return imgBestFit;
}

// Get original image
cv::Mat LaneDetector::getImgOrg(){
    return imageOrg;
};

// Get IPM image
cv::Mat LaneDetector::getImgIPM(){
    return image;
};

// Get hough image
cv::Mat LaneDetector::getHough(){
    return hough;
}

// Get the lines
cv::Vec2f LaneDetector::getLines(){
    return lines;
}

// Get result with best fit line overlayed on original image
cv::Mat LaneDetector::getResult(){
    return result;
}
