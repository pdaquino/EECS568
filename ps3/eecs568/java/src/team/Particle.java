package team;

import java.util.*;

import april.jmat.*;

public class Particle
{
    // List of previous points (trajectory) ... last of which is our current pos
    LinkedList<double[]> trajectory = new LinkedList<double[]>();

    // List of EKFs for landmark observations. Map landmark IDs to EKFs
    HashMap<Integer, EKF> observations = new HashMap<Integer, EKF>();   // XXX Create those EKFs

    double weight = 1.0;  // Particle weight for resampling
    double chi2;    // Chi^2 value for this particle

    public void updateLocation(double[] local_xyt)
    {
        trajectory.add(LinAlg.xytMultiply(trajectory.getLast(), local_xyt));
    }

    // Given an (r,theta), do data association and add appropriate landmark
    // XXX Using known data association for now...get rid of it later
    public void updateLandmark(double[] obs, int id)
    {

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
        for (double[] xyt: trajectory) {
            p.trajectory.add(LinAlg.copy(xyt));
        }

        for (Integer key: observations.keySet()) {
            p.observations.put(key, observations.get(key).copy());  // XXX Copy of those EKFs!
        }

        p.chi2 = chi2;

        return p;
    }
}
