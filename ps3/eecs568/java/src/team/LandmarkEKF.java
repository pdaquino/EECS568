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
            //this.measurementNoise.set(0, 0, 3);
            //this.measurementNoise.set(1, 1, 1);
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

    // XXX Jacobian
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
        residual[1] = MathUtil.mod2pi(theta - h[1]);

        return residual;
    }

    // Get the last residual we calculated
    private double[] getResidual() {
        return residual;
    }
    
    public double getObservationChi2(double r, double theta, double[] robotPose) {
        Matrix Q = this.getMeasurementCovariance(robotPose);
        Matrix Qi = Q.inverse();
        Matrix residual = Matrix.columnMatrix(this.getResidual(r, theta, robotPose));
        return booksExponent(Qi, residual) * -2;
    }
    
    // returns the ln of the probability, using the book's equation
    public double getObservationlogProb(double r, double theta, double[] robotPose) {
        Matrix Q = this.getMeasurementCovariance(robotPose);
        Matrix Qi = Q.inverse();
        Matrix residual = Matrix.columnMatrix(this.getResidual(r, theta, robotPose));
        return booksExponent(Qi, residual);
    }

    // returns ~p(z|x)
    public double update(double r, double theta, double[] robotPose) {
        Matrix H = this.getH(robotPose);
        Matrix Q = this.getMeasurementCovariance(robotPose);
        Matrix Qi = Q.inverse();
        Matrix K = this.covariance.times(H.transpose()).times(Qi);
        Matrix residual = Matrix.columnMatrix(this.getResidual(r, theta, robotPose));

        double w = booksWeight(Q, residual, Qi);

        this.position = Matrix.columnMatrix(position).plus(K.times(residual)).copyAsVector();
        this.covariance = this.covariance.minus(K.times(H).times(this.covariance));

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
        double w = booksNormalizationConst(Q)
                * MathUtil.exp(booksExponent(Qi, residual));
        return w;
    }
    
    protected double booksNormalizationConst(Matrix Q) {
        return (1.0 / Math.sqrt(Q.times(2 * Math.PI).det()));
    }
    
    protected double booksExponent(Matrix Qi, Matrix residual) {
        return -0.5 * residual.transpose().times(Qi).times(residual).get(0);
    }

    protected double edsWeight(double[] obs, double[] robotPose) {
        Matrix J = getH(robotPose);
        Matrix Jt = J.transpose();
        // z0 = zpred - Jf*uf
        double[] z_0 = LinAlg.subtract(getPredictedObservation(robotPose), J.times(position));
        z_0[1] = MathUtil.mod2pi(z_0[1]);
        Matrix y = Matrix.columnMatrix(LinAlg.subtract(obs, z_0));

        Matrix S_zi = measurementNoise.inverse();
        Matrix S_fi = covariance.inverse();
        Matrix u_f = Matrix.columnMatrix(position);
        Matrix S_qi = Jt.times(S_zi).times(J).plus(S_fi);
        Matrix S_q = S_qi.inverse();
        Matrix u_q = S_q.times(Jt.plus(S_zi).times(y).plus(S_fi.times(u_f)));

        // Constants
        double twoPi = 2 * Math.PI;
        double K_z = Math.sqrt(measurementNoise.det()) * twoPi;
        double K_f = Math.sqrt(covariance.det()) * twoPi;
        double K_q = Math.sqrt(S_q.det()) * twoPi;

        double normalization = K_q / (K_z * K_f);
        double exponent = -0.5 * (-1 * ekfChi2(u_q, S_qi) + ekfChi2(y, S_zi) + ekfChi2(u_f, S_fi));

        /*System.out.println("u_f:");
        LinAlg.print(position);
        System.out.println("S_f:");
        covariance.print();
        System.out.println("Chi2 Sum: -" + chi2(u_q, S_qi) + " + " + chi2(y, S_zi) + " + " + chi2(u_f, S_fi) + " = " + exponent * -2);
        System.out.println("Exponent: " + exponent);
        System.out.println("Normalization: " + normalization);*/

        return normalization * MathUtil.exp(exponent);
    }
    
    protected double stupidWeight(double r, double theta, double[] robotPose) {
        Matrix res = Matrix.columnMatrix(getResidual(r, theta, robotPose));
        double exponent = res.transpose().times(measurementNoise.inverse()).times(res).get(0);
        return MathUtil.exp(-0.5*exponent);
    }

    private double ekfChi2(Matrix u, Matrix inverseCov) {
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
        Config config = null;
        try {
            config = new ConfigFile("/home/rgoeddel/class/eecs568/EECS568/ps3/eecs568/config/sim.config");
        } catch (Exception ex) {
            ex.printStackTrace();
            System.exit(1);
        }

        double[] pose = { 3, 2, Math.PI };
        LandmarkEKF ekf = new LandmarkEKF(config, 10, Math.PI/2, pose);
        LandmarkEKF ekf2 = new LandmarkEKF(config, 10, Math.PI/2, pose);

        System.out.println("Position after first observation: ");
        LinAlg.print(ekf.getPosition());
        System.out.println("Covariance after first observation: ");
        ekf.getCovariance().print();

        double w = ekf.update(11, 0.6*Math.PI, pose);
        System.out.println("Position after 2nd observation: ");
        LinAlg.print(ekf.getPosition());
        System.out.println("Covariance after 2nd observation: ");
        ekf.getCovariance().print();
        System.out.println("Weight after 2nd observation: \n"+w);

        System.out.println("===================================");

        w = ekf2.update(10, 0.5*Math.PI, pose);
        System.out.println("Position after 2nd observation: ");
        LinAlg.print(ekf2.getPosition());
        System.out.println("Covariance after 2nd observation: ");
        ekf2.getCovariance().print();
        System.out.println("Weight after 2nd observation: \n"+w);

        w = ekf2.update(10, 0.5*Math.PI, pose);
        System.out.println("Position after 3rd observation: ");
        LinAlg.print(ekf2.getPosition());
        System.out.println("Covariance after 3rd observation: ");
        ekf2.getCovariance().print();
        System.out.println("Weight after 3rd observation: \n"+w);
    }
}
