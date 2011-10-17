package team;

import april.jmat.CSRVec;

/**
 *
 * @author pdaquino
 */
public class ConstraintEdge implements Edge {

    @Override
    public double[] getResidual() {
        return new double[] {0,0,0};
    }

    @Override
    public int getNumberJacobianRows() {
        return 3;
    }

    @Override
    public CSRVec[] getJacobianRows(int stateVectorSize) {
        CSRVec[] vec = new CSRVec[3];
        for(int i = 0; i < 3; i++) {
            vec[i] = new CSRVec(stateVectorSize);
            vec[i].set(i, 42);
        }
        return vec;
    }
    
}
