package rgbdslam;

import java.util.*;
import april.jmat.*;
import april.jmat.geom.*;
import kinect.*;

public class RANSAC
{
    final static double MIN_DIST = 5;  // In meters, min distance for two features to be compared
    final static int DOF = 3;           // Degrees of freedom
    final static int NUM_ITER = 100000;
    final static double BIN_SIZE = 0.1;

    public static double[][] RANSAC(ArrayList<Feature> framea, ArrayList<Feature> frameb)//, ArrayList<double[]> pointsa, ArrayList<double[]> pointsb)
    {
        double[][] bestTransform = new double[3][3];
        if(framea.size() < DOF || frameb.size() < DOF){ 
        return bestTransform;
    }

    Random rand = new Random(83247983);
    double bestConsensus = 0;

    // Get locations of all features
    ArrayList<double[]> pointsa = new ArrayList<double[]>();
    ArrayList<double[]> pointsb = new ArrayList<double[]>();
    for(Feature f: framea){
        pointsa.add(f.getXYZ());
    }
    for(Feature f: frameb){
        pointsb.add(f.getXYZ());
    }
 
    // XXX Not sure this belongs here, could we pass in bins instead?
    BinPoints binb = new BinPoints(pointsb, BIN_SIZE);
    BinPoints bina = new BinPoints(pointsa, BIN_SIZE);       

    for(int iter=0; iter<NUM_ITER; iter++)
    {
        // Get DOF number of different features
        int[] idx = new int[DOF];
        for(int i=0; i<DOF; i++){
            boolean copy;
            do{
                idx[i] = rand.nextInt(frameb.size());
                    
                copy = false;
                for(int j=0; j<i; j++){
                    if( idx[i] == idx[j]) { 
                        copy = true;
                    }
                }
            } while(copy == true);
        }

        // Find corresponding locations and 3D transform
        ArrayList<double[]> cora = new ArrayList<double[]>();
        ArrayList<double[]> corb = new ArrayList<double[]>();

        for(int i=0; i<DOF; i++){
            Feature b = frameb.get(idx[i]);
            corb.add(b.getXYZ());
            cora.add(getCorresponding(framea, b).getXYZ());
        }
        double[][] transform = AlignPoints3D.align(cora, corb);

        // Transform points in a and b and check bins
        // XXX not sure if we want to stick with this - gonna take forever
        int score = 0;
        for (double[] a: pointsa) {
            if (binb.hit(LinAlg.transform(transform, a)))
            score++;
        }
        for (double[] b: pointsb) {
            if (bina.hit(LinAlg.transformInverse(transform, b)))
                score++;
            }
            if (score > bestConsensus) {
                bestConsensus = score;
                bestTransform = transform;
            }
            if (bestConsensus >= .8*(pointsa.size()+pointsb.size())) {
                return bestTransform;
            }
        }
        return bestTransform;
    }

    private static Feature getCorresponding(ArrayList<Feature> features, Feature f)
    {
        double[] fxyz = f.getXYZ();
        double[] featureVec = f.getFeatures();

        // XXX - Not sure about relationship between distance and feature distance
        Feature best = null;
        double bestDist = 1000000;
        double minDiff = 1000000;
        for(Feature fcompare: features){
            if(LinAlg.distance(fxyz, fcompare.getXYZ()) < MIN_DIST){
                if(featureDist(featureVec, fcompare.getFeatures()) < minDiff){
                    best = fcompare;
                    bestDist = LinAlg.distance(fxyz, fcompare.getXYZ());
                    minDiff = featureDist(featureVec, fcompare.getFeatures());
                }
            }
        }
        return best;
    }

    // Returns sum of squared differences for each element in vector
    private static double featureDist(double[] f1, double[] f2)
    {
        assert(f1.length == f2.length);

        double dist = 0;
        for(int i=0; i<f1.length; i++){
            dist += (f1[i] - f2[i])*(f1[i] - f2[i]);
        }

        return dist;
    }


    public static void main(String[] args)
    {
        Random rand = new Random(3948723);
        int numFeatures = 1000;
        ArrayList<Feature> fakeFeatures = new ArrayList<Feature>();
        ArrayList<Feature> fakeRotated = new ArrayList<Feature>();

        double[] xyt = new double[3];
        xyt[0] = .25;
        xyt[1] = .35;
        xyt[2] = Math.PI/3;
        System.out.println("Actual transformation: "+xyt[0]+", "+xyt[1]+", "+xyt[2]);

        for(int i=0; i<numFeatures; i++){
            double[] featureVec = new double[128];
            for(int j=0; j<featureVec.length; j++){
                featureVec[j] = rand.nextDouble();
            }

            double[] xyz = new double[3];
            xyz[0] = rand.nextDouble()*6 - 3;
            xyz[1] = rand.nextDouble()*6 - 3;
            xyz[2] = rand.nextDouble()*6 - 3;
            double[] xyzR = LinAlg.transform(xyt, xyz);

            Feature f = new Feature(xyz, featureVec);
            Feature fR = new Feature(xyzR, featureVec);
            fakeFeatures.add(f);
            fakeRotated.add(fR);
        }
        
        double[][] guess = RANSAC(fakeFeatures, fakeRotated);
    
        double ep = 0.000001; // Acceptable error
        for(int i=0; i<numFeatures; i++){
            double[] original = fakeFeatures.get(i).getXYZ();
            double[] transformed = LinAlg.transform(guess, original);
            double[] rotated = fakeRotated.get(i).getXYZ();

            double diff0 = Math.abs(transformed[0]-rotated[0]);
            double diff1 = Math.abs(transformed[1]-rotated[1]);
            double diff2 = Math.abs(transformed[2]-rotated[2]);
            assert(diff0 < ep && diff1 < ep && diff2 < ep);
        }

    }
}