package rgbdslam;

import kinect.*;

import april.jmat.*;
import april.jmat.geom.*;

import java.lang.Double;
import java.util.*;
import rgbdslam.KdTree.Entry;


/* ICP does no internal downsampling so it assumes that it has been given a
 * down sampled point cloud */
public class RGBDICPNew {

    final static int MAX_ITERATIONS = 50; // maximum number of iteration for ICP
    final static int MULTIPLIER_CAP = 15; // maximum multiplication factor for features.
    final static double THRESHOLD = 0.0005; // threshold for convergence change in normalized error
    final static double DISCARD_D = .5; // threshold for outlier rejection
    private KdTree.SqrEuclid<double[]> kdtree; // kdtree for storing points in B

    // Constructs ICP iterator which can be called to allign ColorPointClouds with this one
    public RGBDICPNew(ColorPointCloud cpcB) {

        ArrayList<double[]> PB = cpcB.points;

        if (PB.size() > 0) {
            kdtree = new KdTree.SqrEuclid<double[]>(3, PB.size()); // points are in 3 space
            // add points
            for (double[] P : PB) {
                kdtree.addPoint(P, P); // since our points are simply points in 3D space
                // we are sorting based on their location in 3D space and the point is what we want back
            }
        } else {
            kdtree = new KdTree.SqrEuclid<double[]>(3, 0); // max size of 0
        }
    }

    // XXX alternate constructor that uses our Voxel represenation for B ?
    // refines initial estimate of an RBT that alligns points in A with points in B
    // in frame B
    public double[][] match(ColorPointCloud cpcA, double[][] rbt, List<DescriptorMatcher.Match> Fmatches) {
        int cntr = 0;
        double curerror = Double.MAX_VALUE / 2; // these values will represent an average error for points
        double preverror = Double.MAX_VALUE;

        // get points stored in colored point clouds
        ArrayList<double[]> PA = cpcA.points;

        // arraylists to store feature points to approximate having good constraints on inlier points
        ArrayList<double[]> ExtraA = new ArrayList<double[]>();
        ArrayList<double[]> ExtraB = new ArrayList<double[]>();
        ArrayList<Double> ExtraW = new ArrayList<Double>();
        //System.out.println("Processing Matches");
        processFMatches(Fmatches, ExtraA, ExtraB, PA.size(), ExtraW);

        /*
        System.out.println("Point Cloud contains: " + PA.size() + " Points.");
        System.out.println("We got " + Fmatches.size() + " feature matches");
        System.out.println("now we have " + ExtraA.size() + " extra points");
         */

        // arraylists to store corresponding points in A and B
        ArrayList<double[]> GoodA = new ArrayList<double[]>();
        ArrayList<double[]> GoodB = new ArrayList<double[]>();
        //System.out.println("Computing correspondances");
        curerror = compCorrespondances(PA, rbt, GoodA, GoodB); // how good was the initial guess just for point cloud
        //System.out.println("Computing Feature Error");
        curerror = 0.5 * curerror + 0.5 * compFeatureError(ExtraA, ExtraB, rbt, ExtraW); // how good was the initial guess the features
        // construct weighting matrix
        ArrayList<Double> Weights = new ArrayList<Double>();
        for (int i = 0; i < GoodA.size(); i++) {
            Weights.add(1.0);
        }
        Weights.addAll(ExtraW); // add these weights at the end to contribute
        GoodA.addAll(ExtraA); // add these guys in so they constribute to the calculation
        GoodB.addAll(ExtraB);
        
        System.out.println("New: Error " + curerror + " and our previous error is " + preverror);

        

        /*
        System.out.println("Now we have " + GoodA.size() + " Points");
        System.out.println("Initial Error " + curerror);
         */

        // itterate until change in error becomes small or reach max iterations
        while (((preverror - curerror) > THRESHOLD) && (cntr < MAX_ITERATIONS)) {
            // use these lists to compute updated RBT
            // http://www.cs.duke.edu/courses/spring07/cps296.2/scribe_notes/lecture24.pdf
            /*
            System.out.println("AlignPoints3DM!");
            System.out.println("Size GoodA : " + GoodA.size());
            System.out.println("Size GoodB : " + GoodB.size());
            System.out.println("Size Weights : " + Weights.size());
            System.out.println("Size of Extra Weights : "+ ExtraW.size());*/
            double[][] Erbt = AlignPoints3DM.align(GoodA, GoodB, Weights);

            // reassign errors
            preverror = curerror;

            // compute new error and new feature correspondances
            GoodA = new ArrayList<double[]>();
            GoodB = new ArrayList<double[]>();
            //System.out.println("Computing correspondances");
            curerror = compCorrespondances(PA, Erbt, GoodA, GoodB); // how good was the guess just for point cloud
            // curerror is the average point distance
            //System.out.println("Computing Feature Error");
            curerror = 0.5 * curerror + 0.5 * compFeatureError(ExtraA, ExtraB, Erbt, ExtraW); // how good was the guess
            // construct weighting matrix
            Weights = new ArrayList<Double>();
            for (int i = 0; i < GoodA.size(); i++) {
                Weights.add(1.0);
            }
            Weights.addAll(ExtraW); // add these weights at the end to contribute
            GoodA.addAll(ExtraA); // add these guys in so they constribute to the calculation
            GoodB.addAll(ExtraB);

            System.out.println("New: Itteration " + cntr + " Error " + curerror + " and our previous error is " + preverror);

            if (curerror < preverror) {
                rbt = Erbt; // new best guess if we got better
            }
            cntr++;
        }
        System.out.println("New Performed " + cntr + " Iterations.");
        //System.out.println("Normalized Error Change: " + (preverror - curerror));
        return rbt;
    }

    // where GoodA will be features from the current image, and GoodB will be filled with features from the previous image
    // assume that we want alpha = 0.5
    // so weight features in 3D space as much as the whole point cloud, should help avoid drift
    private void processFMatches(List<DescriptorMatcher.Match> Fmatches, List<double[]> ExtraA,
            List<double[]> ExtraB, int CPCAsize, List<Double> ExtraW) {
        assert ((ExtraA != null) && (ExtraB != null) && (ExtraW != null)) : "Need non null arrayLists";

        // calculate multiplier to figure out how many of each feature we need
        int multiplier = 0;
        if (Fmatches.size() > 0) {
            multiplier = CPCAsize / Fmatches.size();
        }
        // implement a multiplier cap to progressively shift weight over to point cloud
        // as we recieve fewer features
        if (multiplier > MULTIPLIER_CAP) {
            multiplier = MULTIPLIER_CAP;
        }
        
        for (int i = 0; i < Fmatches.size(); i++) {
            ExtraA.add(Fmatches.get(i).feature1.xyz());
            ExtraB.add(Fmatches.get(i).feature2.xyz());
            ExtraW.add((double) multiplier);
        }
    }

    private double compCorrespondances(ArrayList<double[]> PA, double[][] Erbt,
            ArrayList<double[]> GoodA, ArrayList<double[]> GoodB) {

        assert ((GoodA != null) && (GoodB != null)) : "Warning must pass non null ArrayLists!";
        assert (GoodA.isEmpty() && GoodB.isEmpty()) : "Warning want empty ArrayLists!";

        // apply rbt to points in A to get into frame B
        ArrayList<double[]> PAinB = LinAlg.transform(Erbt, PA);

        double totalError = 0; // for accumulating error
        int cntr = 0; // how many points are we actually using

        // for each transformed point
        for (int index = 0; index < PAinB.size(); index++) {
            // find nearest point in B for each in A using K-D tree
            List<Entry<double[]>> neighbors = kdtree.nearestNeighbor(PAinB.get(index), 1, false);
            if (neighbors.size() > 0) {
                Entry<double[]> BE = neighbors.get(0);
                double[] B = BE.value;
                double d = BE.distance; // euclidean distance between A and B

                // if distance is small enough, then not an outlier
                if (d < DISCARD_D) {
                    // add both to our list of good points
                    GoodA.add(PA.get(index));
                    GoodB.add(B);
                    // add distance to our error estimate
                    totalError = totalError + d;
                    cntr++;
                }
            }
        }
        return totalError / cntr;
    }

    // computes error for the feature correspondances for this estimate of the rigid body transformation
    // uses the weights encoded in ExtraW to increaes the total error computed in this step to reflect the 
    // extra weight we are giving these particles
    private double compFeatureError(ArrayList<double[]> ExtraA, ArrayList<double[]> ExtraB, double[][] Erbt, ArrayList<Double> ExtraW) {

        double totalError = 0;

        // apply rbt to points in A to get into frame B
        ArrayList<double[]> ExtraAinB = LinAlg.transform(Erbt, ExtraA);

        double totW = 0; // total weight
        int cntr = 0; // how many points are we actually using
        for (int i = 0; i < ExtraA.size(); i++) {
            double d = LinAlg.distance(ExtraAinB.get(i), ExtraB.get(i));
            if (d < DISCARD_D) {
                // add distance to our error estimate
                totalError = totalError + d * ExtraW.get(i);
                totW += ExtraW.get(i);
                cntr++;
            }
        }
        if (ExtraA.isEmpty()) {
            return 0;
        } else {          
            return totalError / totW;
        }
    }
}
