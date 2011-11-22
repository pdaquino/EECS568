package rgbdslam;

import java.util.*;
import april.jmat.*;
import april.jmat.geom.*;
import kinect.*;

public class RANSAC
{
    final double MIN_DIST = .5; // In meters

    public RANSAC(ArrayList<Feature> framea, ArrayList<Feature> frameb)
    {
	Random rand = new Random(83247983);
	Feature fa1, fa2, fa3, fb1, fb2, fb3;

	for(int i=0; i<10000; i++)
	{
	    // Get three different features
	    int idx1, idx2, idx3;
            idx1 = rand.nextInt(frameb.size());
            do {
                idx2 = rand.nextInt(frameb.size());
            } while (idx2 == idx1);
            do {
                idx3 = rand.nextInt(frameb.size());
            } while (idx3 == idx1 && idx3 == idx2);

	    fb1 = frameb.get(idx1);
	    fb2 = frameb.get(idx2);
	    fb3 = frameb.get(idx3);

	    fa1 = getCorresponding(framea, fb1);
	    fa2 = getCorresponding(framea, fb2);
	    fa3 = getCorresponding(framea, fb3);

	    ArrayList<double[]> pointsa = new ArrayList<double[]>();
	    ArrayList<double[]> pointsb = new ArrayList<double[]>();
	    pointsa.add(fa1.getLoc());
	    pointsa.add(fa2.getLoc());
	    pointsa.add(fa3.getLoc());
	    pointsb.add(fb1.getLoc());
	    pointsb.add(fb2.getLoc());
	    pointsb.add(fb3.getLoc());

	    double[][] transfrom = AlignPoints3D.align(pointsa, pointsb);
	}
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