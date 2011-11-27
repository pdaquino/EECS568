package rgbdslam;

import java.util.*;
import april.jmat.*;
import april.jmat.geom.*;
import kinect.*;

public class RANSAC
{
    final double MIN_DIST = .5;  // In meters, min distance for two features to be compared
    final int DOF = 3;           // Degrees of freedom
    final int NUM_ITER = 100000;
    final double BIN_SIZE = 0.1;

    public double[][] RANSAC(ArrayList<Feature> framea, ArrayList<Feature> frameb, ArrayList<double[]> pointsa, ArrayList<double[]> pointsb)
    {
        double[][] bestTransform = new double[3][3];
        if(framea.size() < DOF || frameb.size() < DOF){ 
            return bestTransform;
        }

	Random rand = new Random(83247983);
        double bestConsensus = 0;
 
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
                corb.add(b.getLoc());
                cora.add(getCorresponding(framea, b).getLoc());
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
                System.out.printf("score: %d\n", score);
                bestConsensus = score;
                bestTransform = transform;
            }
            if (bestConsensus >= .8*(pointsa.size()+pointsb.size())) {
                return bestTransform;
            }
	}

        return bestTransform;
    }

    private Feature getCorresponding(ArrayList<Feature> features, Feature f)
    {
	double[] fxyz = f.getLoc();
	ArrayList<Double> featureVec = f.getFeatures();

	// XXX - Not sure about relationship between distance and feature distance
	Feature best = null;
	double bestDist = 1000000;
	double minDiff = 1000000;
	for(Feature fcompare: features){
	    if(LinAlg.distance(fxyz, fcompare.getLoc()) < MIN_DIST){
		if(featureDist(featureVec, fcompare.getFeatures()) < minDiff){
		    best = fcompare;
		    bestDist = LinAlg.distance(fxyz, fcompare.getLoc());
		    minDiff = featureDist(featureVec, fcompare.getFeatures());
		}
	    }
	}
	return best;
    }

    // Returns sum of squared differences for each element in vector
    private double featureDist(ArrayList<Double> f1, ArrayList<Double> f2)
    {
	assert(f1.size() == f2.size());

	double dist = 0;
	for(int i=0; i<f1.size(); i++)
	{
	    dist += (f1.get(i) - f2.get(i))*(f1.get(i) - f2.get(i));
        }

	return dist;
    }
}