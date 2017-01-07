//
//  IPM.hpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 04/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
// This class is baed on that

#ifndef IPM_h
#define IPM_h

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>

/*
 * Inverse Perspective Mapping -> The main class used for inverse perspective mapping
 */
class IPM {
    
    private:
        
        // Sizes
        cv::Size m_origSize;
        cv::Size m_dstSize;
        
        // Points
        std::vector<cv::Point2f> m_origPoints;
        std::vector<cv::Point2f> m_dstPoints;
    
        // Homography
        cv::Mat m_H;
        cv::Mat m_H_inv;
        
        // Maps
        cv::Mat m_mapX, m_mapY;
        cv::Mat m_invMapX, m_invMapY;
    
        void createMaps();
    
    public:
    
        IPM( const cv::Size& _origSize, const cv::Size& _dstSize, const std::vector<cv::Point2f>& _origPoints, const std::vector<cv::Point2f>& _dstPoints );
    
        // Apply IPM on points
        cv::Point2d applyHomography(const cv::Point2d& _point, const cv::Mat& _H);
        cv::Point3d applyHomography( const cv::Point3d& _point, const cv::Mat& _H);
        cv::Point2d applyHomography(const cv::Point2d& _point);
        cv::Point3d applyHomography( const cv::Point3d& _point);
        cv::Point2d applyHomographyInv(const cv::Point2d& _point);
        cv::Point3d applyHomographyInv( const cv::Point3d& _point);
        void applyHomography( const cv::Mat& _origBGR, cv::Mat& _ipmBGR, int borderMode = cv::BORDER_CONSTANT);
        void applyHomographyInv( const cv::Mat& _ipmBGR, cv::Mat& _origBGR, int borderMode = cv::BORDER_CONSTANT);
    
        // Draw
        void drawPoints( const std::vector<cv::Point2f>& _points, cv::Mat& _img ) const;
    
        //********************************************************************************************
        //* SETTERS AND GETTERS
        //********************************************************************************************
        cv::Mat getH() const { return m_H; }
    
        cv::Mat getHinv() const { return m_H_inv; }
    
        void getPoints(std::vector<cv::Point2f>& _origPts, std::vector<cv::Point2f>& _ipmPts);
        
    
};

#endif /* IPM_h */


