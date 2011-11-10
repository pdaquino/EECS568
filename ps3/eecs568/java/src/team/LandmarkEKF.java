package team;

import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.jmat.Matrix;
import april.config.*;

/**
 * An EKF that tracks the position and covariance of a feature.
 * @author pdaquino
 */
public class LandmarkEKF {

    int realID = -1;
    private double[] position;
    private Matrix covariance;
    private static Matrix measurementNoise = null;
    private double[] residual;

    public LandmarkEKF(Config config, double r, double phi, double robotPose[]) {
        double angle = MathUtil.mod2pi(robotPose[2] + phi);
        this.position = new double[2];
        this.residual = new double[2];

        position[0] = robotPose[0] + r * Math.cos(angle);
        position[1] = robotPose[1] + r * Math.sin(angle);

        // jacobian of the covariance projection of (r,t) into (x,y)
        double[][] jw = new double[][]{{Math.cos(angle), -r * Math.sin(angle)},
            {Math.sin(angle), r * Math.cos(angle)}};
        Matrix Jw = new Matrix(jw);

        if (this.measurementNoise == null) {
            double[] sigmas = config.requireDoubles("noisemodels.landmarkDiag");

            this.measurementNoise = new Matrix(2, 2);
            this.measurementNoise.set(0, 0, sigmas[0]);
            this.measurementNoise.set(1, 1, sigmas[1]);
        }
        this.covariance = Jw.times(measurementNoise).times(Jw.transpose());
        //Matrix Hi = getH(robotPose).inverse();
        //this.covariance = Hi.times(measurementNoise).times(Hi.transpose());
    }

    public void setID(int id) {
        realID = id;
    }

    public int getRealID() {
        return realID;
    }

    // private copy constructor
    private LandmarkEKF(LandmarkEKF ekf) {
        this.position = LinAlg.copy(ekf.position);
        this.measurementNoise = measurementNoise.copy();
        this.covariance = ekf.covariance.copy();
        this.residual = LinAlg.copy(ekf.residual);
        this.realID = ekf.realID;
    }

    public Matrix getCovariance() {
        return covariance;
    }

    public Matrix getMeasurementCovariance(double[] robotPose) {
        Matrix H = getH(robotPose);

        return H.times(this.covariance).times(H.transpose()).plus(measurementNoise);
    }

    protected Matrix getH(double[] robotPose) {
        double dx = position[0] - robotPose[0];
        double dy = position[1] - robotPose[1];
        double d2 = dx * dx + dy * dy;
        double d = Math.sqrt(d2);
        double[][] H0 = new double[][]{
            {dx / d, dy / d},
            {-dy / d2, dx / d2}};
        Matrix H = new Matrix(H0);
        return H;
    }

    public double[] getPosition() {
        return position;
    }

    // Returns the last residual calculated for this landmark
    public double[] getResidual(double r, double theta, double[] robotPose) {
        double[] h = getPredictedObservation(robotPose);

        residual[0] = r - h[0];
        residual[1] = theta - h[1];

        return residual;
    }

    // Get the last residual we calculated
    private double[] getResidual() {
        return residual;
    }

    // XXX is this right?
    public double getChi2() {
        double[] r = getResidual();
        Matrix P = getCovariance().inverse();
        Matrix rT = Matrix.rowMatrix(r);
        //return rT.times(P).times(r)[0];
        return LinAlg.magnitude(r);
    }

    // returns p(z|x)
    public double update(double r, double theta, double[] robotPose) {
        Matrix H = this.getH(robotPose);
        Matrix Q = this.getMeasurementCovariance(robotPose);
        Matrix Qi = Q.inverse();
        Matrix K = this.covariance.times(H.transpose()).times(Q.inverse());
        Matrix residual = Matrix.columnMatrix(this.getResidual(r, theta, robotPose));

        double w = edsWeight(new double[]{r, theta}, H, robotPose);

        this.position = Matrix.columnMatrix(position).plus(K.times(residual)).copyAsVector();
        this.covariance = this.covariance.minus(K.times(H).times(this.covariance));

        //double w = booksWeight(Q, residual, Qi);

        return w;
    }

    private double[] getPredictedObservation(double[] robotPose) {
        double[] h = new double[2];
        double dx = position[0] - robotPose[0];
        double dy = position[1] - robotPose[1];
        h[0] = Math.sqrt(dx * dx + dy * dy);
        h[1] = MathUtil.mod2pi(Math.atan2(dy, dx) - robotPose[2]);
        return h;
    }

    protected double booksWeight(Matrix Q, Matrix residual, Matrix Qi) {
        double w = 1 / Math.sqrt(Q.times(2 * Math.PI).det())
                * Math.exp(-0.5 * residual.transpose().times(Qi).times(residual).get(0));
        return w;
    }

    protected double edsWeight(double[] obs, Matrix H, double[] robotPose) {
        Matrix J = H;
        Matrix Jt = J.transpose();
        double[] z_0 = LinAlg.subtract(getPredictedObservation(robotPose), J.times(position));
        Matrix y = Matrix.columnMatrix(LinAlg.subtract(obs, z_0));
        Matrix S_zi = measurementNoise.inverse();
        Matrix S_fi = covariance.inverse();
        Matrix u_f = Matrix.columnMatrix(position);
        Matrix S_qi = Jt.times(S_zi).times(J).plus(S_fi);
        Matrix S_q = S_qi.inverse();
        Matrix u_q = S_q.times(Jt.plus(S_zi).times(y).plus(S_fi.times(u_f)));
        double twoPi = 2 * Math.PI;
        double K_z = Math.sqrt(measurementNoise.det()) * twoPi;
        double K_f = Math.sqrt(covariance.det()) * twoPi;
        double K_q = Math.sqrt(S_q.det()) * twoPi;

        double normalization = K_q / (K_z * K_f);
        double exponent = -0.5 * (-1 * chi2(u_q, S_qi) + chi2(y, S_zi) + chi2(u_f, S_fi));

        System.out.println("u_f:");
        LinAlg.print(position);
        System.out.println("S_f:");
        covariance.print();
        System.out.println("Chi2 Sum: -" + chi2(u_q, S_qi) + " + " + chi2(y, S_zi) + " + " + chi2(u_f, S_fi) + " = " + exponent * -2);
        System.out.println("Exponent: " + exponent);
        System.out.println("Normalization: " + normalization);

        return normalization * Math.exp(exponent);
    }

    private double chi2(Matrix u, Matrix inverseCov) {
        Matrix result = u.transpose().times(inverseCov).times(u);
        assert result.getRowDimension() == 1;
        assert result.getColumnDimension() == 1;
        return result.get(0);
    }

    public LandmarkEKF copy() {
        return new LandmarkEKF(this);
    }

    public static void main(String[] args) {
        // Broken now that config file exists
        /*
        double[] pose = { 3, 2, Math.PI };
        LandmarkEKF ekf = new LandmarkEKF(10, Math.PI/2, pose);
        System.out.println("Position after first observation: ");
        LinAlg.print(ekf.getPosition());
        System.out.println("Covariance after first observation: ");
        ekf.getCovariance().print();
        ekf.update(11, 0.6*Math.PI, pose);
        System.out.println("Position after 2nd observation: ");
        LinAlg.print(ekf.getPosition());
        System.out.println("Covariance after 2nd observation: ");
        ekf.getCovariance().print();*/
    }
}
