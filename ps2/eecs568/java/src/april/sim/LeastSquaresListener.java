package april.sim;

import april.sim.Simulator.odometry_t;
import java.awt.*;
import java.util.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import team.*;

public class LeastSquaresListener extends AbstractLeastSquaresListener {

    private HashMap<Integer, LandmarkPose> lmarks = new HashMap<Integer, LandmarkPose>();

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets) {
        addOdometryEdge(odom);
        // Deal with landmarks
        ArrayList<LandmarkPose> recentLmarks = new ArrayList<LandmarkPose>();
        for (Simulator.landmark_t landmark : dets) {
            LandmarkPose lpose;
            if (lmarks.containsKey(landmark.id)) {
                lpose = lmarks.get(landmark.id);
            } else {
                lpose = new LandmarkPose(currentStateVectorSize, latestRobotPose,
                        landmark.obs[0], landmark.obs[1]);
                lpose.setId(landmark.id);
                currentStateVectorSize += lpose.getNumDimensions();
                nodes.add(lpose);
                lmarks.put(landmark.id, lpose);
            }
            recentLmarks.add(lpose);
            LandmarkEdge ledge = new LandmarkEdge(config, landmark.obs[0], landmark.obs[1], latestRobotPose, lpose);
            edges.add(ledge);
        }
        doLeastSquaresUpdate(100, recentLmarks);
    }
}
