package rgbdslam;

import java.util.ArrayList;
import java.util.List;
import rgbdslam.KdTree.Entry;

/**
 * Class that matches feature descriptors between two frames.
 * @author pdaquino
 */
public class DescriptorMatcher {

    private KdTree.SqrEuclid<ImageFeature> kdtree;
    
    /**
     * Represents a match between features in two images. We are matching
     * features in features2 to features1.
     */
    public static class Match {
        public ImageFeature feature1;
        public ImageFeature feature2;
        public double distance;
        public Match(ImageFeature f1, ImageFeature f2, double d) {
            this.feature1 = f1;
            this.feature2 = f2;
            this.distance = d;
        }
    }

    /**
     * Constructs a DescriptorMatcher that matches to the features in features1.
     * @param features1 the features in the 1st image
     */
    public DescriptorMatcher(ArrayList<ImageFeature> features1) {
        if (features1.size() > 0) {
            int descriptorSize = features1.get(0).getDescriptor().length;
            kdtree = new KdTree.SqrEuclid<ImageFeature>(descriptorSize, features1.size());
            
            for(ImageFeature feature : features1) {
                kdtree.addPoint(feature.getDescriptor(), feature);
            }
        } else {
            kdtree = new KdTree.SqrEuclid<ImageFeature>(0, 0);
        }
    }
    
    /**
     * Return the matches of features2 in features1
     * @param features2 the features that are going to be matched in feature1
     * @return a list of matches
     */
    public ArrayList<Match> match(List<ImageFeature> features2) {
        ArrayList<Match> matches = new ArrayList<Match>(features2.size());
        
        for(ImageFeature feature : features2) {
            Entry<ImageFeature> match = kdtree.nearestNeighbor(
                    feature.getDescriptor(), 1, true).get(0);
            matches.add(new Match(match.value, feature, match.distance));
        }
        
        return matches;
    }
    
    
}
