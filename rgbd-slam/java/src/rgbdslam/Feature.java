package rgbdslam;

import java.util.*;

public class Feature
{
    private double[] features;
    private double[] xyz; // XXX - need to get this somehow

    public Feature(double[] xyz_, double[] features_)
    {
        xyz = xyz_;
        features = features_;
    }

    public double[] getFeatures()
    { 
        return features;
    }

    public double[] copyFeatures()
    {
        double[] copyF = new double[features.length];
        System.arraycopy(features, 0, copyF, 0, features.length);

        return copyF;
    }

    public double[] getXYZ()
    {
        return xyz;
    }
}