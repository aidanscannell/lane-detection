//
//  IPM.cpp
//  cv_autonomous_vehicle
//
//  Created by AidanScannell on 04/12/2016.
//  Copyright © 2016 AidanScannell. All rights reserved.
//

#include "IPM.hpp"

using namespace cv;
using namespace std;

/********************************************************************************************
 * INVERSE PERSPECTIVE MAPPING
 ********************************************************************************************
 * This function performs the overall IPM
 * Output -> no output
 * \param _origSize - size of the original points
 * \param _dstSize - size of the destination points
 * \param _origPoints - vector containing the original points
 * \param _dstPoints  - vector containing the destination points
 */
IPM::IPM( const Size& _origSize, const Size& _dstSize, const vector<Point2f>& _origPoints, const vector<Point2f>& _dstPoints ): m_origSize(_origSize), m_dstSize(_dstSize), m_origPoints(_origPoints), m_dstPoints(_dstPoints){
    m_H = getPerspectiveTransform( m_origPoints, m_dstPoints );
    m_H_inv = m_H.inv();
    
    createMaps();
}

/********************************************************************************************
 * DRAW POINTS
 ********************************************************************************************
 * This function draws the input points on the input image
 * Output -> no output
 * \param pts - size of the original points
 * \param image - the input image
 */
void IPM::drawPoints( const vector<Point2f>& pts, Mat& image ) const {
    assert(pts.size() == 4);
    
    line(image, Point(static_cast<int>(pts[0].x), static_cast<int>(pts[0].y)), Point(static_cast<int>(pts[3].x), static_cast<int>(pts[3].y)), CV_RGB( 205,205,0), 2);
    line(image, Point(static_cast<int>(pts[2].x), static_cast<int>(pts[2].y)), Point(static_cast<int>(pts[3].x), static_cast<int>(pts[3].y)), CV_RGB( 205,205,0), 2);
    line(image, Point(static_cast<int>(pts[0].x), static_cast<int>(pts[0].y)), Point(static_cast<int>(pts[1].x), static_cast<int>(pts[1].y)), CV_RGB( 205,205,0), 2);
    line(image, Point(static_cast<int>(pts[2].x), static_cast<int>(pts[2].y)), Point(static_cast<int>(pts[1].x), static_cast<int>(pts[1].y)), CV_RGB( 205,205,0), 2);
    
    for(size_t i = 0; i < pts.size(); i++) {
        circle(image, Point(static_cast<int>(pts[i].x), static_cast<int>(pts[i].y)), 2, CV_RGB(238,238,0), -1);
        circle(image, Point(static_cast<int>(pts[i].x), static_cast<int>(pts[i].y)), 5, CV_RGB(255,255,255), 2);
    }
}


void IPM::getPoints(vector<Point2f>& _origPts, vector<Point2f>& _ipmPts) {
    _origPts = m_origPoints;
    _ipmPts = m_dstPoints;
}

void IPM::applyHomography(const Mat& _inputImg, Mat& _dstImg, int _borderMode) {
    // Generate IPM image from src
    remap(_inputImg, _dstImg, m_mapX, m_mapY, INTER_LINEAR, _borderMode);//, BORDER_CONSTANT, Scalar(0,0,0,0));
}

void IPM::applyHomographyInv(const Mat& _inputImg, Mat& _dstImg, int _borderMode) {
    // Generate IPM image from src
    remap(_inputImg, _dstImg, m_mapX, m_mapY, INTER_LINEAR, _borderMode);//, BORDER_CONSTANT, Scalar(0,0,0,0));
}

Point2d IPM::applyHomography( const Point2d& _point ) {
    return applyHomography( _point, m_H );
}

Point2d IPM::applyHomographyInv( const Point2d& _point ) {
    return applyHomography( _point, m_H_inv );
}

Point2d IPM::applyHomography( const Point2d& _point, const Mat& _H ){
    Point2d ret = Point2d( -1, -1 );
    
    const double u = _H.at<double>(0,0) * _point.x + _H.at<double>(0,1) * _point.y + _H.at<double>(0,2);
    const double v = _H.at<double>(1,0) * _point.x + _H.at<double>(1,1) * _point.y + _H.at<double>(1,2);
    const double s = _H.at<double>(2,0) * _point.x + _H.at<double>(2,1) * _point.y + _H.at<double>(2,2);
    if ( s != 0 )
    {
        ret.x = ( u / s );
        ret.y = ( v / s );
    }
    return ret;
}

Point3d IPM::applyHomography( const Point3d& _point ){
    return applyHomography( _point, m_H );
}

Point3d IPM::applyHomographyInv( const Point3d& _point ){
    return applyHomography( _point, m_H_inv );
}

Point3d IPM::applyHomography( const Point3d& _point, const cv::Mat& _H ){
    Point3d ret = Point3d( -1, -1, 1 );
    
    const double u = _H.at<double>(0,0) * _point.x + _H.at<double>(0,1) * _point.y + _H.at<double>(0,2) * _point.z;
    const double v = _H.at<double>(1,0) * _point.x + _H.at<double>(1,1) * _point.y + _H.at<double>(1,2) * _point.z;
    const double s = _H.at<double>(2,0) * _point.x + _H.at<double>(2,1) * _point.y + _H.at<double>(2,2) * _point.z;
    if ( s != 0 )
    {
        ret.x = ( u / s );
        ret.y = ( v / s );
    }
    else
        ret.z = 0;
    return ret;
}


/********************************************************************************************
 * CREATE MAPS
 ********************************************************************************************
 * This function create the remap images for the specified points
 * Output -> no output
 */
void IPM::createMaps() {
    // Create remap images
    m_mapX.create(m_dstSize, CV_32F);
    m_mapY.create(m_dstSize, CV_32F);
    for( int j = 0; j < m_dstSize.height; ++j ) {
        float* ptRowX = m_mapX.ptr<float>(j);
        float* ptRowY = m_mapY.ptr<float>(j);
        for( int i = 0; i < m_dstSize.width; ++i ) {
            Point2f pt = applyHomography( Point2f( static_cast<float>(i), static_cast<float>(j) ), m_H_inv );
            ptRowX[i] = pt.x;
            ptRowY[i] = pt.y;
        }
    }
    
    m_invMapX.create(m_origSize, CV_32F);
    m_invMapY.create(m_origSize, CV_32F);
    
    for( int j = 0; j < m_origSize.height; ++j ) {
        float* ptRowX = m_invMapX.ptr<float>(j);
        float* ptRowY = m_invMapY.ptr<float>(j);
        for( int i = 0; i < m_origSize.width; ++i ) {
            Point2f pt = applyHomography( Point2f( static_cast<float>(i), static_cast<float>(j) ), m_H );
            ptRowX[i] = pt.x;
            ptRowY[i] = pt.y;
        }
    }
}
