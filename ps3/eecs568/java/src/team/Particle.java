package team;

import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.config.*;

public class Particle {

    Config config;
    // List of previous points (trajectory) ... last of which is our current pos
    ArrayList<double[]> trajectory = new ArrayList<double[]>();
    double[] xyt = new double[3];
    // List of EKFs for landmark observations. Map landmark IDs to EKFs
    HashMap<Integer, LandmarkEKF> observations = new HashMap<Integer, LandmarkEKF>();
    double weight = 1.0;  // Particle weight for resampling

    public Particle(Config config_) {
        this(config_, null);
    }

    // Deep copies another particle, but resets weigth to 1
    private Particle(Particle p) {
        this.config = p.config;
        this.trajectory = new ArrayList<double[]>(p.trajectory.size());
        for (double[] xy : p.trajectory) {
            this.trajectory.add(LinAlg.copy(xy));
        }
        this.xyt = LinAlg.copy(p.xyt);

        for (Integer key : p.observations.keySet()) {
            this.observations.put(key, p.observations.get(key).copy());
        }
        this.weight = 1.0;
    }

    public Particle(Config config_, double[] start_) {
        config = config_;

        if (start_ != null) {
            trajectory.add(LinAlg.resize(start_, 2));
            xyt = LinAlg.copy(start_);
        }
    }

    public void updateLocation(double[] local_xyt) {
        xyt = LinAlg.xytMultiply(xyt, local_xyt);
        trajectory.add(LinAlg.resize(xyt, 2));
    }

    public double[] getPose() {
        return LinAlg.copy(xyt);
    }

    public ArrayList<double[]> getTrajectory() {
        return trajectory;
    }

    public ArrayList<double[]> getLandmarks() {
        ArrayList<double[]> landmarks = new ArrayList<double[]>();
        for (LandmarkEKF ekf : observations.values()) {
            landmarks.add(ekf.getPosition());
        }

        return landmarks;
    }

    public ArrayList<Integer> getIDs() {
        ArrayList<Integer> ids = new ArrayList<Integer>();
        for (LandmarkEKF ekf : observations.values()) {
            ids.add(ekf.getRealID());
        }

        return ids;
    }

    // Return IDs for each of the landmarks in the same order as dets are listed.
    // Pass back new particle weight
    public double associateAndUpdateLandmarks(ArrayList<Simulator.landmark_t> dets) {
        // XXX Data association, currently perfect
        for (Simulator.landmark_t det : dets) {
            double r = det.obs[0];
            double theta = det.obs[1];
            if (!observations.containsKey(det.id)) {
                // add the new EKF, but don't change the weights
                LandmarkEKF ekf = new LandmarkEKF(config, r, theta, xyt);
                ekf.setID(det.id);
                observations.put(det.id, ekf);
            } else {
                double w = observations.get(det.id).update(r, theta, xyt);
                weight *= w;
                System.out.println("> Weight: " + weight + " (" + w + ")");
            }
        }

        return weight;
    }

    public double getWeight() {
        return weight;
    }

    public double getChi2() {
        // The Chi2 is directly associated with the error for each
        // landmark. Sum those errors.
        double chi2 = 0;
        for (LandmarkEKF ekf : observations.values()) {
            chi2 += ekf.getChi2();
        }

        return chi2;
    }

    // Return a sample for this particle (deep copy it but reset weight)
    public Particle getSample() {
        return new Particle(this);
    }
}
