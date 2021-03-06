package rgbdslam.test;

import java.util.*;
import april.jmat.*;
import april.jmat.geom.*;
import rgbdslam.DescriptorMatcher.Match;
import rgbdslam.*;

/**
 *
 * @author pdaquino
 */
public class RANSACTest {

    private static Random rand = new Random(1231241);

    
    /**
     * Test harness. Generates a list of numFeatures matches with outlierRatio outliers.
     * @param args 
     */
    public static void main(String[] args) {
        int numFeatures = 1000;
        double outlierRatio = 0.4;

        List<Match> matches = new ArrayList<Match>();

        double[] xyt = new double[3];
        xyt[0] = .25;
        xyt[1] = .35;
        xyt[2] = Math.PI / 3;
        System.out.println("Actual transformation: " + xyt[0] + ", " + xyt[1] + ", " + xyt[2]);

        List<Match> outliers = new ArrayList<Match>();

        for (int i = 0; i < numFeatures; i++) {
            double[] featureVec = new double[128];
            for (int j = 0; j < featureVec.length; j++) {
                featureVec[j] = rand.nextDouble();
            }


            double[] xyz = getRandom3DPoint();

            ImageFeature f = new ImageFeature(0, 0, featureVec);
            f.setXyz(xyz);

            boolean outlier = rand.nextDouble() < outlierRatio;
            double[] xyzR = !outlier ? LinAlg.transform(xyt, xyz) : getRandom3DPoint();


            ImageFeature fR = new ImageFeature(0, 0, featureVec);
            fR.setXyz(xyzR);
            Match match = new DescriptorMatcher.Match(f, fR, 0);
            matches.add(match);

            if (outlier) {
                outliers.add(match);
            }
        }

        List<Match> inliers = new ArrayList<Match>();
        double[][] guess = RANSAC.RANSAC(matches, inliers);

        double totalError = 0;
        int numOutliersSelected = 0;
        for (Match match : inliers) {
            double[] original = match.feature1.xyz();
            double[] transformed = LinAlg.transform(guess, original);
            double[] rotated = match.feature2.xyz();

            totalError += LinAlg.distance(transformed, rotated);

            if (outliers.contains(match)) {
                numOutliersSelected++;
            }
        }

        System.out.printf("Average error: %f\nNumber of inliers: %d\nNumber of expected inliers: %d\nNumber of false positives: %d\n",
                totalError / inliers.size(), inliers.size(), matches.size() - outliers.size(), numOutliersSelected);
    }

    /**
     * Random (x,y,z) point between 0 and MAX.
     * @return 
     */
    private static double[] getRandom3DPoint() {
        final int MAX = 1000;
        double[] xyz = new double[3];
        xyz[0] = rand.nextDouble() * MAX;
        xyz[1] = rand.nextDouble() * MAX;
        xyz[2] = rand.nextDouble() * MAX;
        return xyz;
    }
}
