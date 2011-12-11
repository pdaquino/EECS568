package rgbdslam;

import april.jmat.LinAlg;
import april.jmat.geom.AlignPoints3D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import rgbdslam.DescriptorMatcher.Match;

/**
 *
 * @author pdaquino
 */
public class AdaptativeRANSAC {

    public static class Output {
        public double[][] rbt;
        public List<DescriptorMatcher.Match> inliers;
        public Output(double[][] rbt, List<DescriptorMatcher.Match> inliers) {
            this.rbt = rbt;
            this.inliers = inliers;
        }
    }
    
    public final static double MAX_SQ_CONSENSUS_DISTANCE = 0.01;
    public final static int MODEL_POINTS = 3;
    
    private static Random rand = new Random(83247983);
    
    public static Output RANSAC(List<DescriptorMatcher.Match> matches) {
        double N = Double.POSITIVE_INFINITY;
        double minOutlierRatio = 1;
        double sampleCount = 0;
        Output bestOutput = null;
        while(N > sampleCount) {
            Output output = sample(matches);
            if(bestOutput == null || output.inliers.size() > bestOutput.inliers.size()) {
                bestOutput = output;
                double outlierRatio = 1- ((double)output.inliers.size()) / matches.size();
                if(outlierRatio < 1) {
                    N = getSampleNumber(0.99, outlierRatio);
                }
            }
            sampleCount++;
        }
        System.out.println("AdaptativeRANSAC performed " + sampleCount + " iterations");
        return maximumLikelihoodEstimation(matches, bestOutput);
    }
    
    private static Output sample(List<DescriptorMatcher.Match> matches) {
        // gets DOF indices out of the whole list of matches
            int[] idx = getRandomIndices(matches.size());

            // Find corresponding locations and 3D transform for the points forming the model
            ArrayList<double[]> cora = new ArrayList<double[]>();
            ArrayList<double[]> corb = new ArrayList<double[]>();

            for (int i = 0; i < MODEL_POINTS; i++) {
                DescriptorMatcher.Match match = matches.get(idx[i]);
                cora.add(match.feature1.xyz());
                corb.add(match.feature2.xyz());
            }

            // compute the RBT from feature1 to feature2
            double[][] transform = AlignPoints3D.align(cora, corb);
            List<DescriptorMatcher.Match> inliers = computeInliers(matches, transform);
            return new Output(transform, inliers);
    }
    
    private static Output maximumLikelihoodEstimation(List<Match> matches, Output output) {
        Output mleOutput = new Output(output.rbt, output.inliers);
        int lastInlierCount = 0;
        do {
            lastInlierCount = mleOutput.inliers.size();
            ArrayList<double[]> cora = new ArrayList<double[]>();
            ArrayList<double[]> corb = new ArrayList<double[]>();
            for(Match m : mleOutput.inliers) {
                cora.add(m.feature1.xyz());
                corb.add(m.feature2.xyz());
            }
            mleOutput.rbt = AlignPoints3D.align(cora, corb);
            mleOutput.inliers = computeInliers(matches, mleOutput.rbt);
        } while(mleOutput.inliers.size() > lastInlierCount);
        return mleOutput;
    }
    
    protected static List<DescriptorMatcher.Match> computeInliers(List<Match> matches, double[][] transform) {
        // we now compute the score for this model. one match votes "yes" if the current RBT makes its
        // feature on the first frame align with the one on the second frame.
        List<DescriptorMatcher.Match> currentInliers = new ArrayList<DescriptorMatcher.Match>(matches.size());
        for (DescriptorMatcher.Match match : matches) {
            double[] predictedFeature2 = LinAlg.transform(transform, match.feature1.xyz());
            double sqdDistance = LinAlg.squaredDistance(predictedFeature2, match.feature2.xyz());
            if (sqdDistance  < MAX_SQ_CONSENSUS_DISTANCE) {
                //System.out.println("D = " + sqdDistance);
                match.xyzDistance = Math.sqrt(sqdDistance);
                currentInliers.add(match);
            }
        }
        return currentInliers;
    }
    
    private static double getSampleNumber(double p, double outlierRatio) {
        return Math.ceil(Math.log(1-p)/Math.log(1 - Math.pow(1-outlierRatio, MODEL_POINTS)));
    }
    
    /**
     * Returns a list of DOF indices between 0 and n.
     * @param n
     * @return
     */
    protected static int[] getRandomIndices(int n) {
        // Get DOF number of different features
        int[] idx = new int[MODEL_POINTS];
        for (int i = 0; i < MODEL_POINTS; i++) {
            boolean copy;
            do {
                idx[i] = rand.nextInt(n);

                copy = false;
                for (int j = 0; j < i; j++) {
                    if (idx[i] == idx[j]) {
                        copy = true;
                    }
                }
            } while (copy == true);
        }
        return idx;
    }
}
