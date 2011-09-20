package team;

import april.jmat.*;

import java.util.*;

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
        }

        Matrix M = new Matrix(P).inverse();
        return LinAlg.dotProduct(w, M.times(w));
    }

    /** Compute a set of points that, when plotted as a curve, would trace out an
        iso-probability contour corresponding to the specified chi^2 value. Generate
        points at one-degree spacings using your method from part D.
    **/
    public ArrayList<double[]> getContour(double chi2)
    {
        //  XXX Write me
        return null;
    }


    public static void main(String args[])
    {
        // Insert your test code here.
    }
}
