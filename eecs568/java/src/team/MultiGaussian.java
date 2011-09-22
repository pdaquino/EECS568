package team;

import april.jmat.*;
import april.vis.*;

import java.util.*;
import java.awt.*;
import javax.swing.*;

/** Represents a multi-variate Gaussian distribution. This is a
 * template for EECS568; you should fill in the methods below.
 **/
public class MultiGaussian
{
    /** The mean **/
    double u[];

    /** The covariance **/
    double P[][];

    /** Create a new MultiGaussian object with covariance P and mean u.
        (This should be trivial)
    **/
    public MultiGaussian(double P[][], double u[])
    {
        // You shouldn't need to modify this.
        this.P = LinAlg.copy(P);
        this.u = LinAlg.copy(u);
    }

    /** Create a new MultiGaussian object that estimates the covariance and mean
        from the provided samples. This should implement your algorithm from A. **/
    public MultiGaussian(ArrayList<double[]> samples)
    {
        // Assumes samples exist
        assert (samples.size() > 0);

        // Initialization
        int d = samples.get(0).length;
        u = new double[d];
        P = new double[d][d];

        // Sample mean
        for (double[] sample: samples) {
            // Assumes samples are all of equal length
            assert (sample.length == d);

            for (int i = 0; i < d; i++) {
                u[i] += sample[i];
            }
        }
        for (int i = 0; i < u.length; i++) {
            u[i] /= samples.size();
        }

        // Sample covariance
        for (int j = 0; j < d; j++) {
            for (int k = 0; k < d; k++) {
                // Calculate value for Pjk
                for (double[] sample: samples) {
                    P[j][k] += ((sample[j] - u[j])*(sample[k] - u[k]));
                }
                P[j][k] /= (samples.size() - 1);
            }
        }
    }

    /** Return the covariance associated with this object. (Trivial). **/
    public double[][] getCovariance()
    {
        // You shouldn't need to modify this.
        return P;
    }

    /** Return the mean associated with this object. (Trivial). **/
    public double[] getMean()
    {
        // You shouldn't need to modify this.
        return u;
    }

    /** Draw a random sample from this distribution. This should implement your
        method from part C.
    **/
    public double[] sample(Random r)
    {
        CholeskyDecomposition cd = new CholeskyDecomposition(new Matrix(P));
        Matrix B = cd.getL();

        double z[] = new double[u.length];
        for (int i = 0; i < z.length; i++) {
            z[i] = r.nextGaussian();
        }

        double[] y = B.times(z);
        assert (y.length == u.length);
        for (int i = 0; i < y.length; i++) {
            y[i] += u[i];
        }

        return y;
    }

    /** Given an observation from the distribution, compute the chi^2 value. This
        is given by (x-u)'inv(M)(x-u)
    **/
    public double chi2(double[] x)
    {
        assert (x.length == u.length);
        double[] w = new double[u.length];

        for (int i = 0; i < w.length; i++) {
            w[i] = x[i] - u[i];
            System.out.printf("%f ", w[i]);
        }
        System.out.println();

        Matrix M = new Matrix(P).inverse();
        return LinAlg.dotProduct(w, M.times(w));
    }

    /** Compute a set of points that, when plotted as a curve, would trace out an
        iso-probability contour corresponding to the specified chi^2 value. Generate
        points at one-degree spacings using your method from part D.
    **/
    public ArrayList<double[]> getContour(double chi2)
    {
        // Only handle the 2D case
        assert (u.length == 2);

        ArrayList<double[]> points = new ArrayList<double[]>();
        Matrix Si = (new Matrix(P)).inverse();

        for (int t = 0; t < 360; t++) {
            double theta = Math.toRadians(t);
            double[] w = new double[] { Math.cos(theta),
                                        Math.sin(theta) };
            double alpha = Math.sqrt(chi2 / LinAlg.dotProduct(w, Si.times(w)));

            // Project point
            points.add(new double[] { u[0] + alpha*Math.cos(theta),
                                      u[1] + alpha*Math.sin(theta) });
        }

        return points;
    }

    // === Comparison checks for doubles, double vectors, and double matrices ===
    static public boolean pass(double v0, double v1)
    {
        System.out.printf("expected: %f\n", v0);
        System.out.printf("actual: %f\n", v1);
        return MathUtil.doubleEquals(v0, v1);
    }

    static public boolean pass(double[] v0, double[] v1) {
        System.out.printf("expected: [");
        for (double d: v0) {
            System.out.printf(" %3.5f", d);
        }
        System.out.printf(" ]\n");
        System.out.printf("actual: [");
        for (double d: v1) {
            System.out.printf(" %3.5f", d);
        }
        System.out.printf(" ]\n");

        return LinAlg.equals(v0, v1, 0.1);
    }

    static public boolean pass(double[][] m0, double[][] m1) {
        Matrix M0 = new Matrix(m0);
        Matrix M1 = new Matrix(m1);
        System.out.println("expected: \n"+M0.toString());
        System.out.println("actual: \n"+M1.toString());

        return M0.equals(M1, 0.1);
    }

    public static void main(String args[])
    {
        // Test 1: Sample Mean/Variance
        // [1 0] [0 1] [1 1]
        // u = [2/3 2/3]
        // P = [1/3  -1/6
        //      -1/6 1/3]
        System.out.println("=====================================\n"+
                           "=== Sample Mean/Cov Test ============\n"+
                           "=====================================");
        double[] eu = new double[] {2.0/3.0, 2.0/3.0};
        double[][] eP = new double[][] { {1.0/3.0, -1.0/6.0},
                                         {-1.0/6.0, 1.0/3.0} };
        ArrayList<double[]> samples = new ArrayList<double[]>();
        samples.add(new double[] {1, 0});
        samples.add(new double[] {0, 1});
        samples.add(new double[] {1, 1});
        MultiGaussian mg = new MultiGaussian(samples);
        System.out.println(pass(eu, mg.getMean()) ? "PASS" : "FAIL");
        System.out.println(pass(eP, mg.getCovariance()) ? "PASS" : "FAIL");



        // Test 2: Random sampling.
        // Procedure: draw 10000 random samples, reconstruct MultiGaussian from
        //            samples, verify u, P values. Repeat several times.
        System.out.println("=====================================\n"+
                           "=== Random Sample Test ==============\n"+
                           "=====================================");
        double[] mu = new double[] {0, 2, 5};
        double[][] Sigma = new double[][] { {2, 0, 0},
                                            {0, 1, 0},
                                            {0, 0, 2} };
        mg = new MultiGaussian(Sigma, mu);
        for (int trial = 0; trial < 5; trial++) {
            samples.clear();
            Random r = new Random();
            for (int i = 0; i < 10000; i++) {
                samples.add(mg.sample(r));
            }
            MultiGaussian sampleMG = new MultiGaussian(samples);
            System.out.println(pass(mu, sampleMG.getMean()) ? "PASS" : "FAIL");
            System.out.println(pass(Sigma, sampleMG.getCovariance()) ? "PASS" : "FAIL");
        }

        // Test 3: Chi^2
        // Procedure: Using the same MultiGaussian as in test 2, calculate some
        //            chi2 values and compare to known (hand-verified) answers
        // 1) x = [0, 0, 0], chi2 = 33/2
        // 2) x = [-1, 2, 1], chi2 = 17/2
        // 3) x = [3, 2, 0], chi2 = 27/2
        // 4) x = [0, 2, 5], chi2 = 0
        System.out.println("=====================================\n"+
                           "=== Chi^2 Test ======================\n"+
                           "=====================================");
        double[] x0 = new double[3];
        double[] x1 = new double[] {-1, 2, 1};
        double[] x2 = new double[] {3, 2, 0};
        double[] x3 = new double[] {0, 2, 5};

        double chi20 = mg.chi2(x0);
        double chi21 = mg.chi2(x1);
        double chi22 = mg.chi2(x2);
        double chi23 = mg.chi2(x3);

        System.out.println(pass(33.0/2.0, chi20) ? "PASS" : "FAIL");
        System.out.println(pass(17.0/2.0, chi21) ? "PASS" : "FAIL");
        System.out.println(pass(34.0/2.0, chi22) ? "PASS" : "FAIL");
        System.out.println(pass(0, chi23) ? "PASS" : "FAIL");

        // Test 4: Plotting contours and points
        JFrame jf = new JFrame("PS1 MultiGaussians");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        jf.setVisible(true);
    }
}
