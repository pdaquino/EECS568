package rgbdslam;

import april.jmat.LinAlg;
import java.util.ArrayList;
import java.util.List;
import kinect.*;
import kinect.Kinect.Frame;
import rgbdslam.DescriptorMatcher.Match;

/**
 *
 * @author pdaquino
 */
public class AlignFrames {

    public static final int DECIMATION_FACTOR = 10;

    private List<ImageFeature> currFeatures, lastFeatures;
    private ColorPointCloud currFullPtCloud, currDecimatedPtCloud;
    private ColorPointCloud lastFullPtCloud, lastDecimatedPtCloud;
    private List<DescriptorMatcher.Match> latestInliers;

    public AlignFrames(Kinect.Frame currFrame, Kinect.Frame lastFrame) {
        this.currFullPtCloud = makeFullPtCloud(currFrame);
        this.currFeatures = extractAndProjectFeatures(currFrame, currFullPtCloud);
        this.currDecimatedPtCloud = makeDecimatedPtCloud(currFrame);

        this.lastFullPtCloud = makeFullPtCloud(lastFrame);
        this.lastFeatures = extractAndProjectFeatures(lastFrame, lastFullPtCloud);
        this.lastDecimatedPtCloud = makeDecimatedPtCloud(lastFrame);
    }

    public AlignFrames(Kinect.Frame currFrame, List<ImageFeature> lastProjectedFeatures,
            ColorPointCloud lastFullPtCloud, ColorPointCloud lastDecimatedPtCloud) {
        this.currFullPtCloud = makeFullPtCloud(currFrame);
        this.currFeatures = extractAndProjectFeatures(currFrame, currFullPtCloud);
        this.currDecimatedPtCloud = makeDecimatedPtCloud(currFrame);


        this.lastFeatures = lastProjectedFeatures;
        this.lastFullPtCloud = lastFullPtCloud;
        this.lastDecimatedPtCloud = lastDecimatedPtCloud;
    }

    public double[][] align() {
        return align(null, null);
    }
    
    public double[][] align(List<Match> allMatches, List<Match> inliers) {
        DescriptorMatcher dm = new DescriptorMatcher(lastFeatures, currFeatures);
        ArrayList<DescriptorMatcher.Match> matches = dm.match();

        if(inliers == null) {
            inliers = new ArrayList<Match>();
        }
        
        double[][] transform = RANSAC.RANSAC(matches, inliers);
        if (transform == null) {
            System.err.println("ERR: Null transformation returned by RANSAC");
            transform = LinAlg.identity(4);
        }
        
        double[][] I = new double[][] {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
        
        ICP icp = new ICP(lastDecimatedPtCloud);

        //transform = new double[][] {{Math.cos(Math.PI/6), 0, Math.sin(Math.PI/6), 0},
        //    {0, 1, 0, 0}, {-Math.sin(Math.PI/6), 0, Math.cos(Math.PI/6), 0},{0,0,0,1}};*/

        transform = icp.match(currDecimatedPtCloud, transform);
        latestInliers = inliers;
        if(allMatches != null) {
            allMatches.addAll(matches);
        }

        return transform;
    }

    public ColorPointCloud getLastDecimatedPtCloud() {
        return lastDecimatedPtCloud;
    }

    public List<ImageFeature> getLastFeatures() {
        return lastFeatures;
    }

    public ColorPointCloud getLastFullPtCloud() {
        return lastFullPtCloud;
    }

    public ColorPointCloud getCurrDecimatedPtCloud() {
        return currDecimatedPtCloud;
    }

    public List<ImageFeature> getCurrFeatures() {
        return currFeatures;
    }

    public ColorPointCloud getCurrFullPtCloud() {
        return currFullPtCloud;
    }

    public List<Match> getLatestInliers() {
        return latestInliers;
    }
   
    private ColorPointCloud makeDecimatedPtCloud(Frame frame) {
        return new ColorPointCloud(frame, DECIMATION_FACTOR);
    }

    private ColorPointCloud makeFullPtCloud(Frame frame) {
        return new ColorPointCloud(frame);
    }

    public static List<ImageFeature> extractAndProjectFeatures(Frame frame, ColorPointCloud fullPtCloud) {
        List<ImageFeature> allFeatures = OpenCV.extractFeatures(frame.argb, Constants.WIDTH);
        List<ImageFeature> features = new ArrayList<ImageFeature>();
        for (ImageFeature fc : allFeatures) {
            fc.setXyz(fullPtCloud.Project(LinAlg.copyDoubles(fc.xy())));
            if (fc.xyz()[2] != -1) {
                features.add(fc);
            }
        }
        return features;
    }
}
