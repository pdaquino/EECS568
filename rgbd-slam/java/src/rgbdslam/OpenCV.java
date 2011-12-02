package rgbdslam;

import java.util.ArrayList;
import java.util.List;

/**
 * This class is a wrapper for OpenCV functions.
 * @author pdaquino
 */
public class OpenCV {
    private native int cvExtractFeatures(
            // in (see cv::GoodFeaturesToTrack)
            int[] image, int imageWidth,
            int maxFeatures, double minQuality, double minDistance,
            int blockSize,
            // out
            int[] featuresX, int[] featuresY, double[] descriptors);

    static {
        System.loadLibrary("opencvwrapper");
    }

    public final static int DESCRIPTOR_SIZE = 128;
    public final static int DEFAULT_MAX_FEATURES = 100;
    public final static double DEFAULT_MIN_QUALITY = 0.01;
    public final static int DEFAULT_MIN_DISTANCY = 1;
    public final static int DEFAULT_BLOCK_SIZE = 3;


    public List<ImageFeature> extractFeatures(int[] img, int width) {
        return extractFeatures(img, width, DEFAULT_MAX_FEATURES,
                DEFAULT_MIN_QUALITY, DEFAULT_MIN_DISTANCY, DEFAULT_BLOCK_SIZE);
    }

    public List<ImageFeature> extractFeatures(int[] img, int width, int maxFeatures,
            double minQuality, int minDistancy, int blockSize) {
        int[] featuresX = new int[maxFeatures];
        int[] featuresY = new int[maxFeatures];
        double[] descriptors = new double[DESCRIPTOR_SIZE*maxFeatures];

        int nFeatures = cvExtractFeatures(img, width, maxFeatures, minQuality,
                minDistancy, blockSize, featuresX, featuresY, descriptors);

        if(nFeatures < 0) {
            throw new OpenCVException("cvExtractFeatures returned " + nFeatures);
        }

        List<ImageFeature> features = new ArrayList<ImageFeature>(nFeatures);
        for(int i = 0; i < nFeatures; i++) {
            int x = featuresX[i];
            int y = featuresY[i];
            double[] descriptor = new double[DESCRIPTOR_SIZE];
            System.arraycopy(descriptors, i*DESCRIPTOR_SIZE, descriptor, 0, DESCRIPTOR_SIZE);
            features.add(new ImageFeature(x, y, descriptor));
        }

        return features;

    }
}
