package team;

import java.util.*;

import april.config.*;
import april.sim.*;
import april.vis.*;

class FastSLAM implements Simulator.Listener
{
    Config config;
    VisWorld vw;

    public void init(Config config_, VisWorld vw_)
    {
        config = config_;
        vw = vw_;
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {

    }
}
