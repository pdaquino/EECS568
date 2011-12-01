#include "OpenCVWrapper.h"

#include <iostream>
#include <vector>
#include <algorithm>

#include <cv.hpp>
#include <highgui.h>

struct JIntArrayElements {
    JNIEnv* env;
    jintArray jarray;
    int* region;
    int length;

    JIntArrayElements(JNIEnv* _env, jintArray _jarray)
    : env(_env), jarray(_jarray),
    region(_env->GetIntArrayElements(_jarray, NULL)),
    length(_env->GetArrayLength(_jarray)) {
    }

    ~JIntArrayElements() {
        env->ReleaseIntArrayElements(jarray, region, 0);
    }
};

struct JDoubleArrayElements {
    JNIEnv* env;
    jdoubleArray jarray;
    double* region;
    int length;

    JDoubleArrayElements(JNIEnv* _env, jdoubleArray _jarray)
    : env(_env), jarray(_jarray),
    region(_env->GetDoubleArrayElements(_jarray, NULL)),
    length(_env->GetArrayLength(_jarray)) {
    }

    ~JDoubleArrayElements() {
        env->ReleaseDoubleArrayElements(jarray, region, 0);
    }
};

JNIEXPORT jint JNICALL Java_rgbdslam_OpenCV_cvExtractFeatures(JNIEnv *env, jobject,
        // In arguments:
        jintArray jimg, // the image, as an array formed by concatenating all rows
        jint imgWidth, // the width of the image( len(jimg)/imgWidth gives the number of rows) );
        jint jmaxFeatures, // the maximum number of features returned
        jdouble jminQuality, // how bad the worst feature is allowed to be, in relation to the best
        jdouble jminDistance, // the minimum distance between two features in the image
        jint jblockSize, // size of the block used to compute the 2nd moment matrix
        // Out arguments:
        jintArray jfeaturesX, // the x coordinates of all features
        jintArray jfeaturesY, // the y coordinates of all features
        jdoubleArray jdescriptors // the concatenation of the 128 double
        // descriptors for each feature
        ) {
    using namespace cv;
    try {
        JIntArrayElements imgPtr(env, jimg);
        int height = imgPtr.length / imgWidth;

        cv::Mat img(height, imgWidth, CV_8UC4, imgPtr.region);

        GoodFeaturesToTrackDetector detector(
                jmaxFeatures,
                jminQuality,
                jminDistance,
                jblockSize);
        std::vector<KeyPoint> features;
        detector.detect(img, features);

        SiftDescriptorExtractor extractor;
        Mat descriptors;
        extractor.compute(img, features, descriptors);

        // write stuff back
        JIntArrayElements featuresX(env, jfeaturesX);
        JIntArrayElements featuresY(env, jfeaturesY);
        JDoubleArrayElements descriptorsArray(env, jdescriptors);

        // sometimes the Sift extractor can add features (https://code.ros.org/trac/opencv/ticket/1130)
        // so ensure we still have at most jmaxFeatures
        int nFeatures = std::min(features.size(), static_cast<size_t> (jmaxFeatures));

        if (nFeatures > featuresX.length || nFeatures > featuresY.length) {
            std::cerr << "opencvwrapper error: feature arrays not large enough for " << nFeatures << " features\n";
            return -1;
        }
        
        int descriptorSize = extractor.descriptorSize();
        if(descriptorSize*nFeatures > descriptorsArray.length) {
            std::cerr << "opencvwrapper error: descriptors array of " << descriptorsArray.length << " elements not large enough for " << nFeatures << " descriptors of " << descriptorSize << " doubles\n";
            return -1;
        }

        for (int i = 0; i < nFeatures; ++i) {
            featuresX.region[i] = features[i].pt.x;
            featuresY.region[i] = features[i].pt.y;
            // copy the descriptor
            Mat descriptor = descriptors.row(i);
            std::copy(descriptor.begin<double>(), descriptor.end<double>(),
                    descriptorsArray.region + i*descriptorSize);
        }
        
        return nFeatures;

    } catch (std::exception& e) {
        std::cerr << "opencvwrapper error: " << e.what() << "\n";
        return -1;
    };
    
    // should never reach this
    return -2;
}
