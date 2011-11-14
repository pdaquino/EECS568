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

        double[] centroid1 = LinAlg.centroid(p1);
        double[] centroid2 = LinAlg.centroid(p2);
        double[] xyta = LinAlg.resize(centroid1, 3);
        double[] xyta_inv = LinAlg.xytInverse(xyta);
        double[] xy = LinAlg.resize(getOptimalTranslation(p1, p2), 3);
        double[] xy_trans = new double[] {xyta[0]+xy[0], xyta[1]+xy[1], 0};
        double theta = getOptimalRotation(p1, p2);
        if (Double.isNaN(theta))
            theta = 0;
        double[] xyt_theta = new double[] {0, 0, theta};

        double[][] trans1 = LinAlg.translate(xyta_inv); // Faking t as z
        double[][] rotate = LinAlg.rotateZ(theta);
        double[][] trans2 = LinAlg.translate(xy_trans);

        LinAlg.timesEquals(trans2, rotate);
        LinAlg.timesEquals(trans2, trans1);
        double[] xyzrpy = LinAlg.matrixToXyzrpy(trans2);
        return new double[] {xyzrpy[0], xyzrpy[1], xyzrpy[5]};
    }

    protected static double getOptimalRotation(ArrayList<double[]> p1, ArrayList<double[]> p2) {
        double M = 0;
        double N = 0;

        double[] centroid1 = LinAlg.centroid(p1);
        double[] centroid2 = LinAlg.centroid(p2);

        for(int i = 0; i < p1.size(); i++) {
            double[] x = LinAlg.copy(p1.get(i));
            x[0] -= centroid1[0];
            x[1] -= centroid1[1];
            double[] y = LinAlg.copy(p2.get(i));
            y[0] -= centroid2[0];
            y[1] -= centroid2[1];

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

    static public void main(String[] args)
    {
        ArrayList<double[]> pointsa = new ArrayList<double[]>();
        ArrayList<double[]> pointsb = new ArrayList<double[]>();
        pointsa.add(new double[] {0,0});
        pointsa.add(new double[] {1,1});
        pointsa.add(new double[] {-1,-1});
        pointsa.add(new double[] {0,2});
        pointsa.add(new double[] {5,-3});
        pointsa.add(new double[] {-2,0});
        double[] xyt = new double[] {-1,3,3*Math.PI/2};
        LinAlg.print(xyt);
        System.out.println("--------");
        for (double[] a: pointsa) {
            pointsb.add(LinAlg.transform(xyt, a));
        }

        double[] guess_xyt = getTransformation(pointsa, pointsb);
        System.out.println("--------");
        LinAlg.print(guess_xyt);
    }
}
