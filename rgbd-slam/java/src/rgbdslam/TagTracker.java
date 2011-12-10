package rgbdslam;

import april.jmat.LinAlg;
import april.jmat.geom.AlignPoints3D;
import april.tag.CameraUtil;
import april.tag.Tag36h11;
import april.tag.TagDetection;
import april.tag.TagDetector;
import april.tag.TagFamily;
import java.util.ArrayList;
import java.util.List;
import kinect.Kinect;
import kinect.Constants;

/**
 *
 * @author pdaquino
 */
public class TagTracker {

    public final static TagFamily TAG_FAMILY = new Tag36h11();
    public final static double[] OPTICAL_CENTER = new double[]{
        kinect.Constants.Crgbx, kinect.Constants.Crgby};
    public final static double TAG_SIZE = 0.131;
    public final static double VERTICES[][] = {{-TagTracker.TAG_SIZE / 2, -TagTracker.TAG_SIZE / 2, 0},
        {TagTracker.TAG_SIZE / 2, -TagTracker.TAG_SIZE / 2, 0},
        {TagTracker.TAG_SIZE / 2, TagTracker.TAG_SIZE / 2, 0},
        {-TagTracker.TAG_SIZE / 2, TagTracker.TAG_SIZE / 2, 0}};
    
    private static List<double[]> verticesList;

    private class Match {

        public TagDetection last;
        public TagDetection curr;

        public Match(TagDetection last, TagDetection curr) {
            assert last.id == curr.id;
            this.last = last;
            this.curr = curr;
        }
    }
    private TagDetector detector = new TagDetector(TAG_FAMILY);
    private Kinect.Frame lastFrame;
    private List<TagDetection> lastDetections;
    private List<Match> matches;

    public TagTracker(Kinect.Frame firstFrame) {
        this.lastFrame = firstFrame;
        this.lastDetections = detector.process(lastFrame.makeRGB(), OPTICAL_CENTER);
        this.detector.segDecimate = true;
        if(verticesList == null) {
            verticesList = new ArrayList<double[]>(4);
            verticesList.add(VERTICES[0]); verticesList.add(VERTICES[1]); verticesList.add(VERTICES[2]);
            verticesList.add(VERTICES[3]);
        }
    }

    public int update(Kinect.Frame currFrame) {
        List<TagDetection> currDetections = detector.process(currFrame.makeRGB(), OPTICAL_CENTER);
        // TagDetection does not implement Comparable so we can't use contains()
        matches = new ArrayList<Match>();
        for (TagDetection currDet : currDetections) {
            for (TagDetection lastDet : lastDetections) {
                if (currDet.id == lastDet.id) {
                    matches.add(new Match(lastDet, currDet));
                }
            }
        }

        this.lastFrame = currFrame;
        this.lastDetections = currDetections;
        System.out.printf("%d tags tracked\n", matches.size());

        return matches.size();
    }

    public List<TagDetection> getDetections() {
        return lastDetections;
    }

    public double[][] getTransformation() {
        // XXX for now just using the first match
        ArrayList<double[]> currVertices = new ArrayList<double[]>(matches.size()*4);
        ArrayList<double[]> lastVertices = new ArrayList<double[]>(matches.size()*4);
        if (!matches.isEmpty()) {
            Match m = matches.get(0);
            double[][] currTransf = homographyToPose(m.curr.homography);
            double[][] lastTransf = homographyToPose(m.last.homography);
            currVertices.addAll(LinAlg.transform(currTransf, verticesList));
            lastVertices.addAll(LinAlg.transform(lastTransf, verticesList));
            return AlignPoints3D.align(currVertices, lastVertices);
        } else {
            return LinAlg.identity(4);
        }
    }

    public static double[][] homographyToPose(double[][] h) {
        return CameraUtil.homographyToPose(Constants.Frgbx, Constants.Frgby, TAG_SIZE, h);
    }
}
