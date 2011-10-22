package team;

import april.jmat.*;
import april.config.*;
import java.util.ArrayList;

public class LandmarkEdge implements Edge
{
    private RobotPose robot;
    private LandmarkPose lmark;

    private double r, theta;

    //static Matrix invSigma = null;
    static double invSigmas[] = null;

    public LandmarkEdge(Config config, double r, double theta, RobotPose robot, LandmarkPose lmark)
    {
        this.r = r;
        this.theta = theta;
        this.robot = robot;
        this.lmark = lmark;

        if (invSigmas == null) {
            int rows = getNumberJacobianRows();
            invSigmas = new double[rows];
            double[] sigmas = config.requireDoubles("noisemodels.landmarkDiag");
            invSigmas[0] = 1.0/sigmas[0];
            invSigmas[1] = 1.0/sigmas[1];
            //invSigma.print();
        }
    }

    public double[] getResidual()
    {
        double dx = lmark.getPosition()[0] - robot.getPosition()[0];
        double dy = lmark.getPosition()[1] - robot.getPosition()[1];
        double robotTheta = robot.getPosition()[2];

        // Actual values
        double btheta = MathUtil.mod2pi(MathUtil.atan2(dy, dx) - robotTheta);
        double br = Math.sqrt(dx*dx + dy*dy);

        // Residual = observed values - expected values
        // Observed means the sensor readings we got
        // Expected means the sensor readings we should be getting if we were
        // where we currently think we are.
        double[] residual = new double[2];
        residual[0] = r - br;
        residual[1] = theta - btheta;

        return residual;
    }
    
    

    public Matrix getJacobian(int stateVectorSize)
    {
        Matrix J = new Matrix(getNumberJacobianRows(), stateVectorSize, Matrix.SPARSE);
        J.setRow(0, getRRow(stateVectorSize));
        J.setRow(1, getThetaRow(stateVectorSize));
        return J;
    }
    
    public ArrayList<double[]> getEndpoints() {
        ArrayList<double[]> endpoints = new ArrayList<double[]>(2);
        endpoints.add(LinAlg.resize(robot.getPosition(), 2));
        endpoints.add(lmark.getPosition());
        return endpoints;
    }

    public Matrix getCovarianceInverse(int nColumnToFill, int nAllRows)
    {
        Matrix SigmaInv = new Matrix(invSigmas.length, nAllRows);
        for(int i = 0; i < invSigmas.length; i++) {
            SigmaInv.set(i, nColumnToFill+i, invSigmas[i]);
        }
        return SigmaInv;
    }

    private CSRVec getThetaRow(int stateVectorSize)
    {
        CSRVec vec = new CSRVec(stateVectorSize);
        double dx = lmark.getPosition()[0] - robot.getPosition()[0];
        double dy = lmark.getPosition()[1] - robot.getPosition()[1];
        double dydx2 = (dy/dx)*(dy/dx);

        int robotIdx = robot.getIndex();
        vec.set(robotIdx, (1.0/(1.0 + dydx2)) * dy/(dx*dx));    // x0
        vec.set(robotIdx+1, (1.0/(1.0 + dydx2)) * -1/(dx));     // y0
        vec.set(robotIdx+2, -1);                                // T0

        int lmarkIdx = lmark.getIndex();
        vec.set(lmarkIdx, (1.0/(1.0 + dydx2)) * -dy/(dx*dx));   // x1
        vec.set(lmarkIdx+1, (1.0/(1.0 + dydx2)) * 1/(dx));      // y1

        return vec;
    }

    private CSRVec getRRow(int stateVectorSize)
    {
        CSRVec vec = new CSRVec(stateVectorSize);
        double dx = lmark.getPosition()[0] - robot.getPosition()[0];
        double dy = lmark.getPosition()[1] - robot.getPosition()[1];
        double dr = Math.sqrt(dx*dx + dy*dy);

        int robotIdx = robot.getIndex();
        vec.set(robotIdx, -dx/dr);      // x0
        vec.set(robotIdx+1, -dy/dr);    // y0
        vec.set(robotIdx+2, 0);         // T0

        int lmarkIdx = lmark.getIndex();
        vec.set(lmarkIdx, dx/dr);       // x1
        vec.set(lmarkIdx+1, dy/dr);     // y1

        return vec;
    }


    public int getNumberJacobianRows() {
        return 2;
    }
}
