package team;

import april.jmat.CSRVec;

/**
 *
 * @author pdaquino
 */
public class ConstraintEdge implements Edge {

    @Override
    public double[] getResidual() {
        return new double[] {0};
    }

    @Override
    public int getNumberJacobianRows() {
        return 1;
    }

    @Override
    public CSRVec[] getJacobianRows(int stateVectorSize) {
        CSRVec[] vec = new CSRVec[1];
        vec[0] = new CSRVec(stateVectorSize);
        vec[0].set(0, 1);
        vec[0].set(1, 1);
        vec[0].set(2, 1);
        return vec;
    }

}
