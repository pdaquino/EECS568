package team;

import april.jmat.LinAlg;
import april.jmat.MathUtil;
import java.util.ArrayList;

/**
 *
 * @author pdaquino
 */
public class RotationFit
{
    // returns the angle (in rad) that approximates p2[i] = Rp1[i], where R
    // is the corresponding rotation matrix
    // assumes that p1[i] matches p2[i]
    public static double fitRotation(ArrayList<double[]> p1, ArrayList<double[]> p2)
    {
        if(p1.size() != p2.size()) {
            throw new IllegalArgumentException();
        }
        
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
    
    public static ArrayList<double[]> applyRotation(double theta, ArrayList<double[]> p) {
        double[] T = new double[] { 0, 0, theta };
        return LinAlg.transform(T, p);
    }
}
