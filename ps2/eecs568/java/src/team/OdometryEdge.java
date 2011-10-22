package team;

import april.jmat.*;
import april.config.*;

/**
 *
 * @author pdaquino
 */
public class OdometryEdge implements Edge{

    // stores references to the nodes before and after the robot moved
    private RobotPose start, end;
    private double dl, dr, b;
    // the rigid body transformation corresponding to this odometry reading
    private double[] zxyt;

    static Matrix invSigma = null;

    public OdometryEdge(Config config, double dl, double dr, double b, RobotPose start,
            RobotPose end) {
        this.b = b;
        this.dl = dl;
        this.dr = dr;
        this.start = start;
        this.end = end;
        zxyt = new double[] {(dr+dl)/2, 0, Math.atan((dr-dl)/b)};
        end.setPosition(getPredictedEndPosition(start));

        if (invSigma == null) {
            int rows = getNumberJacobianRows();
            invSigma = new Matrix(rows, rows, Matrix.SPARSE);
            double[] sigmas = config.requireDoubles("noisemodels.odometryDiag");
            double lateralNoise = .05; // XXX??????????

            invSigma.set(0,0,(1.0/4.0)*(sigmas[0]+sigmas[1]));
            invSigma.set(2,2,(b*b/4.0)*(sigmas[0]+sigmas[1]));
            invSigma.set(0,2,(b/2.0)*(sigmas[0]-sigmas[1]));
            invSigma.set(2,0,(b/2.0)*(sigmas[0]-sigmas[1]));
            invSigma.set(1,1,lateralNoise);

            invSigma = invSigma.inverse();
            //invSigma.print();
        }
    }

    @Override
    public double[] getResidual() {
        double dx = end.getPosition()[0] - start.getPosition()[0];
        double dy = end.getPosition()[1] - start.getPosition()[1];
        double dt = end.getPosition()[2] - start.getPosition()[2];
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.sin(start.getPosition()[2]);

        // Calculate "actual" value
        double[] bxyt = LinAlg.xytInvMul31(start.getPosition(), end.getPosition());

        // Expected values
        /*double bdl = cosTa*dx + sinTa*dy - b*dt/2;
        double bdr = cosTa*dx + sinTa*dy + b*dt/2;

        // Residual = observed values - expected values
        double[] residual = new double[2];
        residual[0] = dl - bdl;
        residual[1] = dr - bdr;*/

        double[] residual = new double[3];
        residual[0] = zxyt[0] - bxyt[0];
        residual[1] = zxyt[1] - bxyt[1];
        residual[2] = zxyt[2] - bxyt[2];
        return residual;
    }

    // XXX
    private double[] getPredictedEndPosition(RobotPose start) {
        return LinAlg.xytMultiply(start.getPosition(), zxyt);
    }

    @Override
    public Matrix getJacobian(int stateVectorSize) {
        Matrix J = new Matrix(getNumberJacobianRows(), stateVectorSize, Matrix.SPARSE);
        J.setRow(0, getXRow(stateVectorSize));
        J.setRow(1, getYRow(stateVectorSize));
        J.setRow(2, getTRow(stateVectorSize));
        return J;
    }

    public Matrix getCovarianceInverse()
    {
        return Matrix.identity(getNumberJacobianRows(), getNumberJacobianRows());
        //return invSigma;
    }

    private CSRVec getDlRow(int stateVectorSize) {
        CSRVec vec = new CSRVec(stateVectorSize);
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.sin(start.getPosition()[2]);
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
        double sinTa = Math.sin(start.getPosition()[2]);
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

    private CSRVec getXRow(int stateVectorSize) {
        CSRVec vec = new CSRVec(stateVectorSize);
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.sin(start.getPosition()[2]);
        double dx = end.getPosition()[0] - start.getPosition()[0];
        double dy = end.getPosition()[1] - start.getPosition()[1];

        int startIdx = start.getIndex();
        vec.set(startIdx, -cosTa);      // x0
        vec.set(startIdx+1, -sinTa);    // y0
        vec.set(startIdx+2, -sinTa*dx + cosTa*dy); // T0

        int endIdx = end.getIndex();
        vec.set(endIdx, cosTa);       // x1
        vec.set(endIdx+1, sinTa);     // y1
        vec.set(endIdx+2, 0);     // T1

        return vec;
    }

    private CSRVec getYRow(int stateVectorSize) {
        CSRVec vec = new CSRVec(stateVectorSize);
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.sin(start.getPosition()[2]);
        double dx = end.getPosition()[0] - start.getPosition()[0];
        double dy = end.getPosition()[1] - start.getPosition()[1];

        int startIdx = start.getIndex();
        vec.set(startIdx, sinTa);       // x0
        vec.set(startIdx+1, -cosTa);    // y0
        vec.set(startIdx+2, sinTa*dy - cosTa*dx);  // T0

        int endIdx = end.getIndex();
        vec.set(endIdx, -sinTa);       // x1
        vec.set(endIdx+1, cosTa);     // y1
        vec.set(endIdx+2, 0);     // T1

        return vec;
    }

    private CSRVec getTRow(int stateVectorSize) {
        CSRVec vec = new CSRVec(stateVectorSize);
        double cosTa = Math.cos(start.getPosition()[2]);
        double sinTa = Math.sin(start.getPosition()[2]);
        double dx = end.getPosition()[0] - start.getPosition()[0];
        double dy = end.getPosition()[1] - start.getPosition()[1];

        int startIdx = start.getIndex();
        vec.set(startIdx, 0);       // x0
        vec.set(startIdx+1, 0);    // y0
        vec.set(startIdx+2, -1);  // T0

        int endIdx = end.getIndex();
        vec.set(endIdx, 0);       // x1
        vec.set(endIdx+1, 0);     // y1
        vec.set(endIdx+2, 1);     // T1

        return vec;
    }

    @Override
    public int getNumberJacobianRows() {
        //return 2;
        return 3;
    }

}
