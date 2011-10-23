package april.sim;

import april.jmat.LinAlg;
import april.sim.Simulator.landmark_t;
import april.sim.Simulator.odometry_t;
import java.util.ArrayList;
import team.LandmarkEdge;
import team.LandmarkPose;
import team.Node;

/**
 *
 * @author pdaquino
 */
public class LeastSquaresNoAssocListener extends AbstractLeastSquaresListener {

    @Override
    public void update(odometry_t odom, ArrayList<landmark_t> landmarks) {
        addOdometryEdge(odom);
        ArrayList<LandmarkPose> recentLmarks = handleLandmarks(landmarks);
        doLeastSquaresUpdate(currentStateVectorSize, recentLmarks);
    }

    /**
     * Return all the landmark nodes that are within a certain distance of the
     * observation
     * @param obs
     * @return 
     */
    public ArrayList<LandmarkPose> getPossibleMatches(landmark_t obs) {
        LandmarkPose tentativeLmark = new LandmarkPose(0, latestRobotPose,
                obs.obs[0], obs.obs[1]);
        ArrayList<LandmarkPose> possibleMatches = new ArrayList<LandmarkPose>();
        for (Node node : nodes) {
            if (!(node instanceof LandmarkPose)) {
                continue;
            }
            LandmarkPose lmarkPose = (LandmarkPose) node;
            double distance = LinAlg.distance(tentativeLmark.getPosition(),
                    lmarkPose.getPosition());
            if (distance < 5) {
                possibleMatches.add(lmarkPose);
            }
        }
        return possibleMatches;
    }

    private ArrayList<LandmarkPose> handleLandmarks(ArrayList<landmark_t> landmarks) {
        ArrayList<LandmarkPose> recentLmarks = new ArrayList<LandmarkPose>();
        for (landmark_t lmark : landmarks) {
            recentLmarks.add(handleLandmark(lmark));
        }
        return recentLmarks;
    }

    private LandmarkPose handleLandmark(landmark_t lmark) {
        LandmarkPose match = getBestMatch(lmark);
        if (match == null) {
            match = new LandmarkPose(currentStateVectorSize, latestRobotPose,
                    lmark.obs[0], lmark.obs[1]);
            match.setId(lmark.id);
            currentStateVectorSize += match.getNumDimensions();
            nodes.add(match);
        }
        LandmarkEdge matchingEdge = new LandmarkEdge(config, lmark.obs[0],
                lmark.obs[1], latestRobotPose, match);
        edges.add(matchingEdge);
        return match;
    }

    private LandmarkPose getBestMatch(landmark_t lmark) {
        // we are going to do some update iterations, which are messing up
        // potentially all nodes. we need to keep a back up
        // (call updateNodesPosition later!)
        double[] originalStateVector = this.getStateVector();
        double originalChi2 = this.getChi2();

        LandmarkPose bestOneSoFar = null;
        double lowestChi2 = 0;
        ArrayList<LandmarkPose> possibleMatches = getPossibleMatches(lmark);
        System.out.println("Testing " + possibleMatches.size() + " posisble matches");
        for (LandmarkPose candidate : possibleMatches) {
            System.out.print("\tLandmark " + candidate.getId() + " is a matching candidate... ");
            LandmarkEdge tentativeEdge = new LandmarkEdge(config, lmark.obs[0],
                    lmark.obs[1], latestRobotPose, candidate);
            edges.add(tentativeEdge);
            doLeastSquaresUpdate(10, null);
            if (bestOneSoFar != null) {
                String debug = ("Chi2 = " + this.getChi2() + "/" + lowestChi2);
                if (this.getChi2() < lowestChi2) {
                    bestOneSoFar = candidate;
                    lowestChi2 = this.getChi2();
                    System.out.println("match (" + debug + ")");
                } else {
                    System.out.println("fail (" + debug + ")");
                }
            } else {
                double cutoff = Math.max(1.2*originalChi2, originalChi2+1);
                String debug = ("Chi2 = " + this.getChi2() + "/" + cutoff);
                if (this.getChi2() < cutoff) {
                    bestOneSoFar = candidate;
                    lowestChi2 = this.getChi2();
                    System.out.println("match (" + debug + ")");
                } else {
                    System.out.println("fail (" + debug + ")");
                }
            }
            // reset state
            this.chi2 = originalChi2;
            updateNodesPosition(originalStateVector);
            edges.remove(edges.size() - 1);
            assert LinAlg.equals(originalStateVector, this.getStateVector(), 0.000001);
        }
        if (bestOneSoFar == null) {
            System.out.println("Could not match observation with any LandmarkPose.");
        } else {
            if(bestOneSoFar.getId() == lmark.id) {
                System.out.println("CORRECT association with landmark " + bestOneSoFar.getId());
            } else {
                System.out.println("WRONG association with landmark " + bestOneSoFar.getId() + " - should be " + lmark.id);
            }
        }
        return bestOneSoFar;
    }
}
