package team;

import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.jmat.Matrix;

/**
 * An EKF that tracks the position and covariance of a feature.
 * @author pdaquino
 */
public class LandmarkEKF {

    private double[] position;
    private Matrix covariance;
    private static Matrix measurementNoise = null;

    public LandmarkEKF(double r, double phi, double robotPose[]) {
        double angle = MathUtil.mod2pi(robotPose[2] + phi);
        this.position = new double[2];
        position[0] = robotPose[0] + r * Math.cos(angle);
        position[1] = robotPose[1] + r * Math.sin(angle);
        // jacobian of the covariance projection of (r,t) into (x,y)
        Matrix Jw = new Matrix(2, 2);
        Jw.set(0, 0, Math.cos(angle));
        Jw.set(0, 1, -r * Math.sin(angle));
        Jw.set(1, 0, Math.sin(angle));
        Jw.set(1, 1, r * Math.cos(angle));
        if (this.measurementNoise == null) {
            this.measurementNoise = new Matrix(2, 2);
            // XXX maybe read this form a config file instead
            this.measurementNoise.set(0, 0, 3);
            this.measurementNoise.set(1, 1, 1);
        }
        this.covariance = Jw.times(measurementNoise).times(Jw.transpose());
    }
    
    // private copy constructor
    private LandmarkEKF(double[] position, Matrix covariance) {
        this.position = position;
        this.covariance = covariance;
    }

    public Matrix getCovariance() {
        return covariance;
    }

    public double[] getPosition() {
        return position;
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
        LinAlg.print(h);
        double[] res = new double[2];
        res[0] = r - h[0];
        res[1] = theta - h[1];
        this.position = Matrix.columnMatrix(position).plus(K.times(Matrix.columnMatrix(res))).copyAsVector();
        this.covariance = this.covariance.minus(K.times(H).times(this.covariance));
    }

    private double[] getPredictedObservation(double[] robotPose) {
        double[] h = new double[2];
        double dx = position[0] - robotPose[0];
        double dy = position[1] - robotPose[1];
        h[0] = Math.sqrt(dx * dx + dy * dy);
        // XXX shouldnt we use atan2?
        h[1] = MathUtil.mod2pi(Math.atan2(dy, dx) - robotPose[2]);
        return h;
    }
    
    public LandmarkEKF copy() {
        return new LandmarkEKF(LinAlg.copy(position), covariance.copy());
    }
    
    public static void main(String[] args) {
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
        ekf.getCovariance().print();
    }
}
