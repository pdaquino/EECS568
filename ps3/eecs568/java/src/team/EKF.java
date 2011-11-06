package team;

import java.util.*;

import april.jmat.*;

public class EKF
{
    // Track covariance over time (SigmaX)
    Matrix sigmaX;
    double[] state; // x,y

    public EKF(double[] state_, double[][] sigma_)
    {
        state = LinAlg.copy(state_);
        sigmaX = new Matrix(sigma_);
    }




}
