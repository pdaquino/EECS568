package team;

import april.jmat.CSRVec;
import april.jmat.Vec;

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
        double[] endPose = end.getPosition();
        double deltaX = endPose[0];
        double deltaT = endPose[2];
        
        double expectedDl = deltaX - (b*deltaT)/2;
        double expectedDr = deltaX + b*deltaT/2;
        
        double[] residual = new double[2];
        residual[0] = expectedDl - dl;
        residual[1] = expectedDr - dr;
        return residual;
        
    }

    private double[] getPredictedEndPosition(RobotPose start) {
        double[] newPosition = new double[3];
        //newPosition[0] = start.getPosition()[0] + (dr+dl)/2;
        //newPosition[1] = start.getPosition()[1];
        //newPosition[2] = start.getPosition()[2] + Math.atan((dr - dl)/b);
        newPosition[0] = (dr+dl)/2;
        newPosition[1] = 0;
        newPosition[2] = Math.atan((dr - dl)/b);
        return newPosition;
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
        int startIdx = start.getIndex();
        //vec.set(startIdx, 1);
        //vec.set(startIdx+2, -b/2);
        int endIdx = end.getIndex();
        vec.set(endIdx, 1);
        vec.set(endIdx+2, -b/2);
        return vec;
    }
    
    private CSRVec getDrRow(int stateVectorSize) {
        CSRVec vec = new CSRVec(stateVectorSize);
        int startIdx = start.getIndex();
        //vec.set(startIdx, 1);
        //vec.set(startIdx+2, b/2);
        int endIdx = end.getIndex();
        vec.set(endIdx, 1);
        vec.set(endIdx+2, b/2);
        return vec;
    }

    @Override
    public int getNumberJacobianRows() {
        return 2;
    }
    
}
