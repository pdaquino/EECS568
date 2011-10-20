package team;

import april.jmat.*;

/**
 *
 * @author pdaquino
 */
public interface Edge {
    double[] getResidual();
    int getNumberJacobianRows();
    Matrix getJacobian(int stateVectorSize);
    Matrix getCovarianceInverse();
}
