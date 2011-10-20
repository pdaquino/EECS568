package team;

import april.jmat.*;

public class LandmarkEdge implements Edge
{
    private RobotPose robot;
    private LandmarkPose lmark;

    private double r, theta;

    public LandmarkEdge(double r, double theta, RobotPose robot, LandmarkPose lmark)
    {
        this.r = r;
        this.theta = theta;
        this.robot = robot;
        this.lmark = lmark;
    }

    public double[] getResidual()
    {
        double dx = lmark.getPosition()[0] - robot.getPosition()[0];
        double dy = lmark.getPosition()[1] - robot.getPosition()[1];
        double robotTheta = robot.getPosition()[2];

        // Expected values
        double btheta = MathUtil.mod2pi(MathUtil.atan2(dy, dx) - robotTheta);
        double br = Math.sqrt(dx*dx + dy*dy);

        // Residual = observed values - expected values
        double[] residual = new double[2];
        residual[0] = r - br;
        residual[1] = theta - btheta;

        return residual;
    }

    public CSRVec[] getJacobianRows(int stateVectorSize)
    {
        CSRVec[] vecs = new CSRVec[2];
        vecs[0] = getRRow(stateVectorSize);
        vecs[1] = getThetaRow(stateVectorSize);
        return vecs;
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
