package rgbdslam;

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
            int[] featuresX, int[] featuresY, double[][] descriptors);
    
    static {
        System.loadLibrary("opencvwrapper");
    }
    
    public void extractFeatures(int[] img, int width, int maxFeatures) {
        //int[] img = { 1, 2, 3, 4 };
        int[] featuresX = new int[maxFeatures];
        int[] featuresY = new int[maxFeatures];
        
        int ret = cvExtractFeatures(img, width, 20, 0.01, 1, 3, featuresX, featuresY, new double[1][]);
        
        for(int i = 0; i < featuresX.length; i++) {
            System.out.println("(" + featuresX[i] + ", " + featuresY[i] + ")");
        }
        
        System.out.println(ret);
    }
}
