package team;

import april.jmat.*;

/**
 *
 * @author pdaquino
 */
public class ConstraintEdge implements Edge {

    Node pose;
    double[] pin;

    static double[] invSigmas = null;

    public ConstraintEdge(Node pose, double[] pin)
    {
        this.pose = pose;
        this.pin = pin;

        if (invSigmas == null) {
            int size = getNumberJacobianRows();
            invSigmas = new double[size];
            for (int i = 0; i < size; i++) {
                invSigmas[i] = 100000;
            }
        }
    }

    @Override
    public double[] getResidual() {
        double[] residual = new double[getNumberJacobianRows()];
        double[] pos = pose.getPosition();

        assert(pin.length == pos.length);
        for (int i = 0; i < residual.length; i++) {
            residual[i] = pin[i] - pos[i];
        }

        return residual;
    }

    @Override
    public int getNumberJacobianRows() {
        return pose.getNumDimensions();
    }

    @Override
    public Matrix getJacobian(int stateVectorSize) {
        int rows = getNumberJacobianRows();
        Matrix J = new Matrix(rows, stateVectorSize, Matrix.SPARSE);
        int idx = pose.getIndex();
        for (int i = 0; i < rows; i++) {
            J.set(i, idx+i, 1);
        }
        return J;
    }

    public Matrix getCovarianceInverse(int nColumnToFill, int nAllRows)
    {
        Matrix SigmaInv = new Matrix(invSigmas.length, nAllRows);
        for(int i = 0; i < invSigmas.length; i++) {
            SigmaInv.set(i, nColumnToFill+i, invSigmas[i]);
        }
        return SigmaInv;
    }

}
