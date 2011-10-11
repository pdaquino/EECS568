package april.sim;

import java.awt.*;
import java.util.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;

public class LeastSquaresListener implements Simulator.Listener
{
    VisWorld vw;
    Config config;

    double xyt[] = new double[3]; // Current rigid body transform

    ArrayList<double[]> trajectory = new ArrayList<double[]>(); // Estimated trajectory
    HashMap<Integer, Integer> lmark2state = new HashMap<Integer, Integer>();
    double baseline; // Robot baseline in [m]

    public void init(Config config_, VisWorld vw_)
    {
        config = config_;
        vw = vw_;

        baseline = config.requireDouble("robot.baseline_m");
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {
        // XXX Do our magic here
        // Look through our landmarks. If we haven't seen them before, add
        // them to our state vector and record their position
        for (Simulator.landmark_t lmark: dets) {
            if (!lmark2state.containsKey(lmark.id))
                lmark2state.put(lmark.id, 0); // XXX For now, just drop an entry in
        }

        // Draw our trajectory and detections
        drawStuff();
    }

    public void drawStuff()
    {
        // Draw stuff here
    }
}
