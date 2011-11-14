package team;

import april.jmat.LinAlg;
import april.jmat.MathUtil;
import java.util.ArrayList;

/**
 *
 * @author pdaquino
 */
public class TransformationFit
{
    // Returns the xyt transformation (rotation and translation) that best aligns
    // the points
    // assumes that p1[i] matches p2[i]
    public static double[] getTransformation(ArrayList<double[]> p1, ArrayList<double[]> p2)
    {
        if(p1.size() != p2.size()) {
            throw new IllegalArgumentException();
        }
        if(p1.isEmpty()) {
            return new double[3];
        }
        
        double[] xy = getOptimalTranslation(p1, p2);
        double theta = getOptimalRotation(p1, p2);
        return new double[] { xy[0], xy[1], Double.isNaN(theta) ? 0 : theta };
    }

    protected static double getOptimalRotation(ArrayList<double[]> p1, ArrayList<double[]> p2) {
        double M = 0;
        double N = 0;
        
        for(int i = 0; i < p1.size(); i++) {
            double[] x = p1.get(i);
            double[] y = p2.get(i);
            M += x[0]*y[1] - x[1]*y[0];
            N += x[0]*y[0] + x[1]*y[1];
        }
        
        return MathUtil.atan2(M, N);
    }

    // moves p1 to align with p2
    private static double[] getOptimalTranslation(ArrayList<double[]> p1, ArrayList<double[]> p2) {
        double[] centroid1 = LinAlg.centroid(p1);
        double[] centroid2 = LinAlg.centroid(p2);
        return LinAlg.subtract(centroid2, centroid1);
    }

}
