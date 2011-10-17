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

    double baseline; // Robot baseline in [m]

    public void init(Config config_, VisWorld vw_)
    {
        config = config_;
        vw = vw_;

        baseline = config.requireDouble("robot.baseline_m");
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {
        // Magic stuff here

        // Draw our trajectory and detections
        drawStuff();
    }

    public void drawStuff()
    {
        // Draw stuff here
    }
}
