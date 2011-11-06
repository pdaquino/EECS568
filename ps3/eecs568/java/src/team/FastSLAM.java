package team;

import java.awt.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.vis.*;

public class FastSLAM implements Simulator.Listener
{
    Config config;
    VisWorld vw;

    // Placeholder
    double[] xyt = new double[3];

    // Config vals
    double baseline;

    public void init(Config config_, VisWorld vw_)
    {
        config = config_;
        vw = vw_;

        baseline = config.requireDouble("robot.baseline_m");
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {
        xyt = LinAlg.xytMultiply(xyt, new double[]{(odom.obs[0] + odom.obs[1]) /2, 0,
                                                   Math.atan((odom.obs[1] - odom.obs[0])/baseline)});
        drawStuff();
    }

    // Draw our own updates to screen
    void drawStuff()
    {
        // Render the robot
        {
            VisWorld.Buffer vb = vw.getBuffer("robot-local");
            VisRobot robot = new VisRobot(new Color(160, 30, 30));

            double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
                                           0, 0, xyt[2]};
            vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
            vb.swap();
        }
    }
}
