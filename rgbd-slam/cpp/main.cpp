#include <iostream>
#include <vector>
#include <algorithm>
#include <ctime>
#include <sstream>

#include "cv.hpp"
#include "highgui.h"

#include "FeatureDetector.hpp"

using namespace cv;

struct PlotPoint {
    cv::Mat& im;

    PlotPoint(Mat & im_) : im(im_) {
    }

    void operator()(const KeyPoint & p) {
        circle(im, p.pt, 1, CV_RGB(255, 0, 0));
    }
};

void write(Mat& im, const std::string& text) {
    putText(im, text, Point2f(0, im.size().height - 5), FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0, 0, 255));
}




void runDetectorOnFeed() {
     VideoCapture cap(0); // open the default camera
    if (!cap.isOpened()) // check if we succeeded
        return;

    time_t start, end;
    time(&start);
    Mat edges;
    int counter = 0;
    const std::string saveFilename = "savedImg.jpg";

    cv::GoodFeaturesToTrackDetector baseDetector(100, 0.01, 1);
    ::FeatureDetector<cv::GoodFeaturesToTrackDetector> detector(baseDetector);
    for (;;) {
        Mat frame;
        cap >> frame; // get a new frame from camera
        std::vector<KeyPoint> features;
        detector.detect(frame, features);

        PlotPoint plotter(frame);
        std::for_each(features.begin(), features.end(), plotter);
        
        time(&end);
        double sec = difftime(end, start);
        std::cout << sec << std::endl;
        ++counter;
        double fps = counter / sec;
        std::stringstream ss;
        ss << fps << " fps";
        write(frame, ss.str());
        cv::imshow("This is Michigan, for God's sake!", frame);
        int key = waitKey(30);
        if(key == 115) { imwrite(saveFilename, frame); }
        else if(key >= 0) { break; }
    }
    // the camera will be deinitialized automatically in VideoCapture destructor   
}

bool compareKeyPoint(const KeyPoint& k1, const KeyPoint& k2) {
    return k1.pt == k2.pt;
}

void compareImages() {
    Mat rn1 = imread("RN_1.jpg");
    Mat rn2 = imread("RN_2.jpg");
    if(rn1.data == NULL) { std::cerr << "oops! file not found\n"; return; }
    
    cv::GoodFeaturesToTrackDetector baseDetector(100, 0.01, 1);
    ::FeatureDetector<cv::GoodFeaturesToTrackDetector> detector(baseDetector);
    
    std::vector<KeyPoint> features1, features2;
    detector.detect(rn1, features1);
    detector.detect(rn2, features2);
    
    std::cout << "detected "<< features1.size() << "\n";
    if( std::equal(features1.begin(), features1.end(), features2.begin(), compareKeyPoint)) {
        std::cout << "feature sets are equal\n";
    } else {
        std::cout << "feature sets are not equal\n";
    }
    
    SiftDescriptorExtractor siftExtractor;
    Mat descriptors1, descriptors2;
    siftExtractor.compute(rn1, features1, descriptors1);
    siftExtractor.compute(rn2, features2, descriptors2);
    
    BruteForceMatcher<SL2<float> > matcher;
    std::vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    
    for(std::vector<DMatch>::const_iterator it = matches.begin(); it != matches.end(); it++) { 
        std::cout << it ->distance << "\n";
    }
    
    Mat outImage;
    drawMatches(rn1, features1, rn2, features2, matches, outImage);
    imshow("Buck the Fuckeyes", outImage);
    waitKey(0);
}

int main() {
    //runDetectorOnFeed();
    compareImages();
    return 0;
}