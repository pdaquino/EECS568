package team;

import april.jmat.CSRVec;

/**
 * 
 * @author pdaquino
 */
public interface Edge {
    double[] getResidual();
    int getNumberJacobianRows();
    CSRVec[] getJacobianRows(int stateVectorSize);
    
}
