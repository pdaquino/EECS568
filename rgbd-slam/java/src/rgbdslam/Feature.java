package rgbdslam;

import java.util.*;

public class Feature
{
    private ArrayList<Double> features;
    private double[] xyz; // XXX - need to get this somehow

    public Feature(double[] xyz_, ArrayList<Double> features_)
    {
	xyz = xyz_;
	features = features_;
    }

    public ArrayList<Double> getFeatures()
    { 
	return features;
    }

    public ArrayList<Double> copyFeatures()
    {
	ArrayList<Double> newf = new ArrayList<Double>();
	for(Double f: features){
	    newf.add(f);
	}
	return newf;
    }

    public double[] getLoc()
    {
	return xyz;
    }
}