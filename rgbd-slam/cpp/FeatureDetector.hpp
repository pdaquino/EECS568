/* 
 * File:   FeatureDetector.hpp
 * Author: pdaquino
 *
 * Created on November 30, 2011, 10:23 PM
 */

#ifndef FEATUREDETECTOR_HPP
#define	FEATUREDETECTOR_HPP

#include "cv.hpp"

template<typename Detector>
class FeatureDetector {
    Detector m_detector;
public:
    FeatureDetector(const Detector& detector) : m_detector(detector) { }
    FeatureDetector() {}
    void detect(cv::Mat& im, std::vector<cv::KeyPoint>& features) {
        cv::Mat bwim;
        cv::cvtColor(im, bwim, CV_BGR2GRAY);
        m_detector.detect(bwim, features);
    }
};

#endif	/* FEATUREDETECTOR_HPP */

