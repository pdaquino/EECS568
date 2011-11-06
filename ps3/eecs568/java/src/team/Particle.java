package team;

import java.util.*;

import april.jmat.*;
import april.sim.*;

public class Particle
{
    // List of previous points (trajectory) ... last of which is our current pos
    ArrayList<double[]> trajectory = new ArrayList<double[]>();

    // List of EKFs for landmark observations. Map landmark IDs to EKFs
    HashMap<Integer, LandmarkEKF> observations = new HashMap<Integer, LandmarkEKF>();

    double weight = 1.0;  // Particle weight for resampling
    double chi2;    // Chi^2 value for this particleA

    public Particle()
    {
        this(null);
    }

    public Particle(double[] start_)
    {
        if (start_ != null)
            trajectory.add(LinAlg.copy(start_));
    }

    public void updateLocation(double[] local_xyt)
    {
        trajectory.add(LinAlg.xytMultiply(getPose(), local_xyt));
    }

    public double[] getPose() {
        assert(trajectory.size() > 0);
        return trajectory.get(trajectory.size()-1);
    }

    public ArrayList<double[]> getTrajectory() {
        ArrayList<double[]> traj = new ArrayList<double[]>();
        for (double[] xyt: trajectory)
            traj.add(LinAlg.resize(xyt, 2));
        return traj;
    }

    // Return IDs for each of the landmarks in the same order as dets are listed.
    // Pass back new particle weight
    public double associateAndUpdateLandmarks(ArrayList<Simulator.landmark_t> dets)
    {
        // XXX Data association, currently perfect
        ArrayList<Integer> ids = new ArrayList<Integer>();
        for (Simulator.landmark_t det: dets) {
            ids.add(det.id);
        }

        // Update the landmark
        for (int i = 0; i < ids.size(); i++) {
            updateLandmark(dets.get(i).obs, ids.get(i));
        }

        // Calculate new weight
        double w = 0;
        for (int i = 0; i < ids.size(); i++) {
            double[] r = observations.get(ids.get(i)).getResidual(dets.get(i).obs[0], dets.get(i).obs[1], getPose());
            Matrix P = observations.get(ids.get(i)).getCovariance();

            w += (1.0/Math.sqrt(2*Math.PI*P.det()))*Math.exp(-0.5*LinAlg.dotProduct(P.inverse().transposeTimes(r), r));
        }

        if (w != 0)
            updateWeight(w);

        return weight;
    }

    // Given an (r,theta), do data association and add appropriate landmark
    // Return the residual after the update
    public double[] updateLandmark(double[] obs, int id)
    {
        if(observations.containsKey(id)) {
            observations.get(id).update(obs[0], obs[1], getPose());
        } else {
            observations.put(id, new LandmarkEKF(obs[0], obs[1], getPose()));
        }
        return observations.get(id).getResidual(obs[0], obs[1], getPose());
    }

    // XXX Some way to get landmark position estimates back
    //
    public void updateWeight(double factor)
    {
        weight *= factor;
    }

    public void normalize(double normalizationFactor)
    {
        weight /= normalizationFactor;
    }

    public double getWeight()
    {
        return weight;
    }

    // XXX Chi2?????

    public double getChi2()
    {
        assert(false);
        return 0;
    }

    // Return a sample for this particle (deep copy it but reset weight)
    public Particle getSample()
    {
        Particle p = new Particle();
        weight = 1.0;
        for (double[] xyt: trajectory) {
            p.trajectory.add(LinAlg.copy(xyt));
        }

        for (Integer key: observations.keySet()) {
            p.observations.put(key, observations.get(key).copy());
        }

        p.chi2 = chi2;

        return p;
    }
}
