package rgbdslam;

import java.util.ArrayList;

/**
 * This class is a wrapper for OpenCV functions.
 * @author pdaquino
 */
public class OpenCV {
    private static native int cvExtractFeatures(
            // in (see cv::GoodFeaturesToTrack)
            int[] image, int imageWidth,
            int maxFeatures, double minQuality, double minDistance,
            int blockSize,
            // out
            int[] featuresX, int[] featuresY, double[] descriptors);
    private static native int cvGetDescriptorSize();

    static {
        System.loadLibrary("opencvwrapper");
    }

    public final static int DEFAULT_MAX_FEATURES = 500;
    public final static double DEFAULT_MIN_QUALITY = 0.01;
    public final static int DEFAULT_MIN_DISTANCY = 1;
    public final static int DEFAULT_BLOCK_SIZE = 3;


    public static ArrayList<ImageFeature> extractFeatures(int[] img, int width) {
        return extractFeatures(img, width, DEFAULT_MAX_FEATURES,
                DEFAULT_MIN_QUALITY, DEFAULT_MIN_DISTANCY, DEFAULT_BLOCK_SIZE);
    }

    public static ArrayList<ImageFeature> extractFeatures(int[] img, int width, int maxFeatures,
            double minQuality, int minDistancy, int blockSize) {
        int[] featuresX = new int[maxFeatures];
        int[] featuresY = new int[maxFeatures];
        int descriptorSize = cvGetDescriptorSize();
        double[] descriptors = new double[descriptorSize*maxFeatures];

        int nFeatures = cvExtractFeatures(img, width, maxFeatures, minQuality,
                minDistancy, blockSize, featuresX, featuresY, descriptors);

        if(nFeatures < 0) {
            throw new OpenCVException("cvExtractFeatures returned " + nFeatures);
        }

        ArrayList<ImageFeature> features = new ArrayList<ImageFeature>(nFeatures);
        for(int i = 0; i < nFeatures; i++) {
            int x = featuresX[i];
            int y = featuresY[i];
            double[] descriptor = new double[descriptorSize];
            System.arraycopy(descriptors, i*descriptorSize, descriptor, 0, descriptorSize);
            features.add(new ImageFeature(x, y, descriptor));
        }

        return features;

    }
}
