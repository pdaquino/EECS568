package april.jmat;

import java.util.*;
import april.util.*;

/** Miscellaneous math utilities like mod2pi and fast exp functions. **/
public class MathUtil
{
    static double epsilon = 0.000000001;
    static double twopi_inv = 0.5/Math.PI;
    static double twopi = 2.0*Math.PI;

    // only good for positive numbers.
    static private double mod2pi_pos(double vin)
    {
        double q = vin * twopi_inv + 0.5;
        int qi = (int) q;

        return vin - qi*twopi;
    }

    /** Ensure that v is [-PI, PI] **/
    static public double mod2pi(double vin)
    {
        double v;

        if (vin < 0)
            v = -mod2pi_pos(-vin);
        else
            v = mod2pi_pos(vin);

        // Validation test:
        //	if (v < -Math.PI || v > Math.PI)
        //		System.out.printf("%10.3f -> %10.3f\n", vin, v);

        return v;
    }

    /** Returns a value of v wrapped such that ref and v differ by no
     * more +/-PI
     **/
    static public double mod2pi(double ref, double v)
    {
        return ref + mod2pi(v-ref);
    }

    /** Returns true if the two doubles are within a small epsilon of
     * each other.
     **/
    static public boolean doubleEquals(double a, double b)
    {
        return Math.abs(a-b)<epsilon;
    }

    static public int clamp(int v, int min, int max)
    {
        if (v<min)
            v=min;
        if (v>max)
            v=max;
        return v;
    }

    static public double clamp(double v, double min, double max)
    {
        if (v<min)
            v=min;
        if (v>max)
            v=max;
        return v;
    }

    public static final double square(double x)
    {
        return x*x;
    }

    public static final int sign(double v)
    {
        if (v >= 0)
            return 1;
        return -1;
    }

    /** Quickly compute e^x for all x.

        Accuracy for x>0:
        x<0.5, absolute error < .0099
        x>0.5, relative error 0.36%

        For x<0, we internally compute the reciprocal form; error is
        magnified.

        This approximation is also monotonic.

    **/
    public static final double exp(double xin)
    {
        if (xin>=0)
            return exp_pos(xin);

        return 1/(exp_pos(-xin));
    }

    /** Quickly compute e^x for positive x.
     **/
    protected static final double exp_pos(double xin)
    {
        // our algorithm: compute 2^(x/log(2)) by breaking exponent
        // into integer and fractional parts.  The integer part can be
        // done with a bit shift operation. The fractional part, which
        // has bounded magnitude, can be computed with a polynomial
        // approximation. We then multiply together the two parts.

        // prevent deep recursion that would just return INF anyway...
        // e^709 > Double.MAX_VALUE;
        if (xin>709)
            return Double.MAX_VALUE;

        if (xin>43) // recursively handle values which would otherwise blow up.
	    {
            // the value 43 was determined emperically
            return 4727839468229346561.4744575*exp_pos(xin-43);
	    }

        double x = 1.44269504088896*xin; // now we compute 2^x
        int wx = (int) x; // integer part
        double rx = x-wx;    // fractional part

        rx*=0.69314718055995; // scale fractional part by log(2)

        double b = 1L<<wx; // 2^integer part
        double rx2 = rx*rx;
        double rx3 = rx2*rx;
        double rx4 = rx3*rx;

        double r = 1+rx+rx2/2+rx3/6+rx4/24; // polynomial approximation for bounded rx.

        return b*r;
    }

    // returns [-PI,PI]
    // accurate within ~0.25 degrees
    public static final double atan2(double y, double x)
    {
        double atn = atan(y/x);

        if (y>=0)
	    {
            if (x>=0)
                return atn;
            return Math.PI+atn;
	    }
        if (x>=0)
	    {
            return atn;
	    }
        return -Math.PI+atn;
    }

    /** returns [-PI/2, PI/2]
        accurate within 0.014 degrees
    **/
    public static final double atan(double x)
    {
        if (Math.abs(x) <= 1)
            return atan_mag1(x);
        if (x < 0)
            return -Math.PI/2-atan_mag1(1/x);
        else
            return Math.PI/2-atan_mag1(1/x);
    }

    // returns reasonable answers for |x|<=1.
    protected static final double atan_mag1(double x)
    {
        // accuracy = 0.26814 degrees
        //	return x/(1+0.28087207802773*x*x);

        if (true) {
            if (Math.abs(x) > 1)
                System.out.printf("ATAN_MAG1: %15f\n", x);

            final double p0 = -0.000158023363661;
            final double p1 = 1.003839939589617;
            final double p2 = -0.016224975245612;
            final double p3 = -0.343317496147292;
            final double p4 = 0.141501628812858;

            double a = Math.abs(x);
            double a2 = a*a;

            double y = p0 + p1*a + p2*a2 + p3*(a2*a) + p4*(a2*a2);

            if (x < 0)
                return -y;
            return y;
        } else {
            double xx = x*x;

            // accuracy = 0.10550 degrees (according to matlab)
            return (0.00182789418543 + 0.97687229491851*x + 0.00087659977713*xx)/
                (0.99499024627366 + 0.00228262896304*x + 0.25288677429562*xx);
        }
    }
}
