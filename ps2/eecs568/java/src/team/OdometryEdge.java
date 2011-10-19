package team;

import april.jmat.*;

/**
 *
 * @author pdaquino
 */
public class OdometryEdge implements Edge{

    // stores references to the nodes before and after the robot moved
    private RobotPose start, end;
    private double dl, dr, b;

    public OdometryEdge(double dl, double dr, double b, RobotPose start,
            RobotPose end) {
        this.b = b;
        this.dl = dl;
        this.dr = dr;
        this.start = start;
        this.end = end;
        end.setPosition(getPredictedEndPosition(start));
    }

    @Override
    public double[] getResidual() {
        double dx = end.getPosition()[0] - start.getPosition()[0];
        double dy = end.getPosition()[1] - start.getPosition()[1];
        double dt = end.getPosition()[2] - start.getPosition()[2];
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.sin(start.getPosition()[2]);

        // Expected values
        double bdl = cosTa*dx + sinTa*dy - b*dt/2;
        double bdr = cosTa*dx + sinTa*dy + b*dt/2;

        // Residual = observed values - expected values
        double[] residual = new double[2];
        residual[0] = dl - bdl;
        residual[1] = dr - bdr;
        return residual;
    }

    // XXX
    private double[] getPredictedEndPosition(RobotPose start) {
        return LinAlg.xytMultiply(start.getPosition(),
                                  new double[] {(dr+dl)/2, 0, Math.atan((dr-dl)/b)});
    }

    @Override
    public CSRVec[] getJacobianRows(int stateVectorSize) {
        CSRVec[] vecs = new CSRVec[2];
        vecs[0] = getDlRow(stateVectorSize);
        vecs[1] = getDrRow(stateVectorSize);
        return vecs;
    }

    private CSRVec getDlRow(int stateVectorSize) {
        CSRVec vec = new CSRVec(stateVectorSize);
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.cos(start.getPosition()[2]);
        double dx = end.getPosition()[0] - start.getPosition()[0];
        double dy = end.getPosition()[1] - start.getPosition()[1];
        int startIdx = start.getIndex();
        vec.set(startIdx, -cosTa);    // x0
        vec.set(startIdx+1, -sinTa);  // y0
        vec.set(startIdx+2, cosTa*dy - sinTa*dx + b/2); // T0
        int endIdx = end.getIndex();
        vec.set(endIdx, cosTa);       // x1
        vec.set(endIdx+1, sinTa);     // y1
        vec.set(endIdx+2, -b/2);    // T1
        return vec;
    }

    private CSRVec getDrRow(int stateVectorSize) {
        CSRVec vec = new CSRVec(stateVectorSize);
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.cos(start.getPosition()[2]);
        double dx = end.getPosition()[0] - start.getPosition()[0];
        double dy = end.getPosition()[1] - start.getPosition()[1];
        int startIdx = start.getIndex();
        vec.set(startIdx, -cosTa);    // x0
        vec.set(startIdx+1, -sinTa);  // y0
        vec.set(startIdx+2, cosTa*dy - sinTa*dx - b/2); // T0
        int endIdx = end.getIndex();
        vec.set(endIdx, cosTa);       // x1
        vec.set(endIdx+1, sinTa);     // y1
        vec.set(endIdx+2, b/2);     // T1
        return vec;
    }

    @Override
    public int getNumberJacobianRows() {
        return 2;
    }

}
