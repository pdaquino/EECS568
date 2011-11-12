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
    //HashMap<Integer, LandmarkEKF> observations = new HashMap<Integer, LandmarkEKF>();
    ArrayList<LandmarkEKF> observations = new ArrayList<LandmarkEKF>();
    double chi2 = 0.0;
    double logProb = 0.0;
    double weight = 1.0;  // Particle weight for resampling
    // weight below which we consider that the observation is a new feature
    final double p0 = 0.9;
    int wrongClosureCounter = 0;

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

//        for (Integer key : p.observations.keySet()) {
//            this.observations.put(key, p.observations.get(key).copy());
//        }
        for (LandmarkEKF ekf : p.observations) {
            this.observations.add(ekf.copy());
        }
        this.weight = 1.0;
        this.chi2 = p.chi2;
        this.logProb = p.logProb;
        this.wrongClosureCounter = p.wrongClosureCounter;
    }

    public Particle(Config config_, double[] start_) {
        config = config_;

        if (start_ != null) {
            trajectory.add(LinAlg.resize(start_, 2));
            xyt = LinAlg.copy(start_);
        }
    }

    public void updateLocation(double[] local_xyt, double chi2) {
        xyt = LinAlg.xytMultiply(xyt, local_xyt);
        trajectory.add(LinAlg.resize(xyt, 2));
        this.chi2 += chi2;
    }

    public double[] getPose() {
        return LinAlg.copy(xyt);
    }

    public ArrayList<double[]> getTrajectory() {
        return trajectory;
    }

    public ArrayList<double[]> getLandmarks() {
        ArrayList<double[]> landmarks = new ArrayList<double[]>();
        for (LandmarkEKF ekf : observations) {
            landmarks.add(ekf.getPosition());
        }

        return landmarks;
    }

    public ArrayList<LandmarkEKF> getLandmarkEKFs() {
        return observations;
    }

    public ArrayList<Integer> getIDs() {
        ArrayList<Integer> ids = new ArrayList<Integer>();
        for (LandmarkEKF ekf : observations) {
            ids.add(ekf.getRealID());
        }

        return ids;
    }

    // Pass back new particle weight
    public double associateAndUpdateLandmarks(ArrayList<Simulator.landmark_t> dets) {
        // XXX Data association, currently perfect
        for (Simulator.landmark_t det : dets) {
            double r = det.obs[0];
            double theta = det.obs[1];
            LandmarkEKF matchingEKF = getBestCorrespondence(det);
            if (matchingEKF == null) {
                // add the new EKF, but don't change the weights
                LandmarkEKF ekf = new LandmarkEKF(config, r, theta, xyt);
                ekf.setID(det.id);
                observations.add(ekf);
            } else {
                if (det.id != matchingEKF.getRealID()) {
                    this.wrongClosureCounter++;
                }
                double obsLogProb = matchingEKF.getObservationlogProb(r, theta, xyt);
                logProb += obsLogProb;
                this.chi2 += matchingEKF.getObservationChi2(r, theta, xyt);
                double w = matchingEKF.update(r, theta, xyt);
                weight *= w;
            }
        }
        return weight;
    }

    // Returns the EKF that best matches the observation. If there are no good
    // matches, return null (that means we think it's a new feature).
    private LandmarkEKF getBestCorrespondence(Simulator.landmark_t det) {
        // the best match is the one that maximizes the weight of the particle
        // if the best weight < p0, then assume it's a new feature
        double bestWeight = this.p0;
        LandmarkEKF bestEKF = null;
        for (LandmarkEKF ekf : observations) {
            double weight = ekf.getObservationWeight(det.obs[0], det.obs[1], xyt);
            if (weight > bestWeight) {
                bestWeight = weight;
                bestEKF = ekf;
            }
        }
        return bestEKF;
    }

    public double getWeight() {
        return weight;
    }

    public double normalizeWeight(double n) {
        weight /= n;
        return weight;
    }

    public double getChi2() {
        return chi2;
    }

    public double getLogProb() {
        return this.logProb;
    }

    public void addLogProb(double logProb) {
        this.logProb += logProb;
    }

    // Return a sample for this particle (deep copy it but reset weight)
    public Particle getSample() {
        return new Particle(this);
    }

    public int getNumWrongClosures() {
        return this.wrongClosureCounter;
    }
}
