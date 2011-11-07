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
        double[][] jw = new double[][] {{ Math.cos(angle), -r*Math.sin(angle)},
                                        { Math.sin(angle), r*Math.cos(angle)}};
        Matrix Jw = new Matrix(jw);

        if (this.measurementNoise == null) {
            double[] sigmas = config.requireDoubles("noisemodels.landmarkDiag");

            this.measurementNoise = new Matrix(2, 2);
            this.measurementNoise.set(0, 0, sigmas[0]);
            this.measurementNoise.set(1, 1, sigmas[1]);
        }
        this.covariance = Jw.times(measurementNoise).times(Jw.transpose());
    }

    public void setID(int id)
    {
        realID = id;
    }

    public int getRealID()
    {
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

    public Matrix getLinearizedCovariance(double r, double theta, double[] robotPose) {
        double dx = position[0] - robotPose[0];
        double dy = position[1] - robotPose[1];
        double d2 = dx * dx + dy * dy;
        double d = Math.sqrt(d2);

        double[][] H0 = new double[][] {{dx/d, dy/d},
                                        {-dy/d2, dy/d2}};
        Matrix H = new Matrix(H0);

        return H.times(this.covariance).times(H.transpose()).plus(measurementNoise);
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
    public double[] getResidual()
    {
        return residual;
    }

    // XXX is this right?
    public double getChi2()
    {
        double[] r = getResidual();
        Matrix P = getCovariance().inverse();

        return LinAlg.dotProduct(P.transposeTimes(r), r);
    }

    public void update(double r, double theta, double[] robotPose) {
        double dx = position[0] - robotPose[0];
        double dy = position[1] - robotPose[1];
        double d2 = dx * dx + dy * dy;
        double d = Math.sqrt(d2);

        Matrix H = new Matrix(2, 2);
        H.set(0, 0, dx / d);
        H.set(0, 1, dy / d);
        H.set(1, 0, -dy / d2);
        H.set(1, 1, dx / d2);

        Matrix S = H.times(this.covariance).times(H.transpose()).plus(measurementNoise);
        Matrix K = this.covariance.times(H.transpose()).times(S.inverse());
        double[] h = getPredictedObservation(robotPose);
        //LinAlg.print(h);

        assert(residual != null);

        residual[0] = r - h[0];
        residual[1] = theta - h[1];

        this.position = Matrix.columnMatrix(position).plus(K.times(Matrix.columnMatrix(residual))).copyAsVector();
        this.covariance = this.covariance.minus(K.times(H).times(this.covariance));
    }

    private double[] getPredictedObservation(double[] robotPose) {
        double[] h = new double[2];
        double dx = position[0] - robotPose[0];
        double dy = position[1] - robotPose[1];
        h[0] = Math.sqrt(dx*dx + dy*dy);
        h[1] = MathUtil.mod2pi(Math.atan2(dy, dx) - robotPose[2]);
        return h;
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
