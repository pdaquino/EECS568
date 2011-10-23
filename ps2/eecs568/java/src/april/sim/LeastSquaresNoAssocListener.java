package april.sim;

import april.jmat.LinAlg;
import april.sim.Simulator.landmark_t;
import april.sim.Simulator.odometry_t;
import java.util.*;
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
    public ArrayList<PossibleMatch> getPossibleMatches(landmark_t obs) {
        LandmarkPose tentativeLmark = new LandmarkPose(0, latestRobotPose,
                obs.obs[0], obs.obs[1]);
        ArrayList<PossibleMatch> possibleMatches = new ArrayList<PossibleMatch>();
        for (Node node : nodes) {
            if (!(node instanceof LandmarkPose)) {
                continue;
            }
            LandmarkPose lmarkPose = (LandmarkPose) node;
            double distance = LinAlg.distance(tentativeLmark.getPosition(),
                    lmarkPose.getPosition());
            if (distance < 5) {
                possibleMatches.add(new PossibleMatch(obs, lmarkPose));
            }
        }
        return possibleMatches;
    }

    private ArrayList<LandmarkPose> handleLandmarks(ArrayList<landmark_t> landmarks) {
        ArrayList<LandmarkPose> recentLmarks = new ArrayList<LandmarkPose>();
        ArrayList<PossibleMatch> possibleMatches = new ArrayList<PossibleMatch>();
        for (landmark_t lmark : landmarks) {
            possibleMatches.addAll(getPossibleMatches(lmark));
        }
        calculateChi2(possibleMatches);
        if (possibleMatches.size() == 0) {
            for (landmark_t lmark: landmarks) {
                LandmarkPose lpose = new LandmarkPose(currentStateVectorSize, latestRobotPose,
                                                      lmark.obs[0], lmark.obs[1]);
                lpose.setId(lmark.id);
                currentStateVectorSize += lpose.getNumDimensions();
                nodes.add(lpose);
                PossibleMatch pm = new PossibleMatch(lmark, lpose);
                possibleMatches.add(pm);
            }
        }

        while (possibleMatches.size() > 0) {
            PossibleMatch bestMatch = possibleMatches.get(0);

            // If our best match isn't very good, create a new landmark,
            // else use the one we're trying
            double chi2 = this.getChi2();
            double cutoff = Math.max(1.2*chi2, chi2+1);
            String debug = ("Chi2 = " + bestMatch.chi2 + "/" + cutoff);
            if (bestMatch.chi2 < cutoff) {
                System.out.println("match (" + debug + ")");
            } else {
                bestMatch.pose = new LandmarkPose(currentStateVectorSize, latestRobotPose,
                        bestMatch.lmark.obs[0], bestMatch.lmark.obs[1]);
                bestMatch.pose.setId(bestMatch.lmark.id);
                currentStateVectorSize += bestMatch.pose.getNumDimensions();
                nodes.add(bestMatch.pose);
                System.out.println("fail (" + debug + ")");
            }

            // Add the edge
            LandmarkEdge ledge = new LandmarkEdge(config, bestMatch.lmark.obs[0],
                    bestMatch.lmark.obs[1], latestRobotPose, bestMatch.pose);
            edges.add(ledge);

            // Add the landmark to the rendering list
            recentLmarks.add(bestMatch.pose);

            // Prune out entries with same landmark id as match
            ArrayList<PossibleMatch> removals = new ArrayList<PossibleMatch>();
            for (PossibleMatch pm: possibleMatches) {
                if (pm.lmark.id == bestMatch.lmark.id) {
                    removals.add(pm);
                    continue;
                } else if (pm.pose.uniqueId() == bestMatch.pose.uniqueId()) {
                    removals.add(pm);
                    continue;
                }
            }
            possibleMatches.removeAll(removals);
        }
        return recentLmarks;
    }

    private void calculateChi2(ArrayList<PossibleMatch> possibleMatches)
    {
        for (PossibleMatch pm: possibleMatches) {
            double[] originalStateVector = this.getStateVector();
            double originalChi2 = this.getChi2();
            LandmarkEdge tentativeEdge = new LandmarkEdge(config, pm.lmark.obs[0],
                    pm.lmark.obs[1], latestRobotPose, pm.pose);
            edges.add(tentativeEdge);
            doLeastSquaresUpdate(10, null);
            pm.chi2 = this.getChi2();

            // reset state
            this.chi2 = originalChi2;
            updateNodesPosition(originalStateVector);
            edges.remove(edges.size() - 1);
            assert LinAlg.equals(originalStateVector, this.getStateVector(), 0.000001);
        }
        Collections.sort(possibleMatches);
    }

    /*private LandmarkPose handleLandmark(landmark_t lmark) {
        LandmarkPose match = getBestMatch(lmark);
        if (match == null) {
            match = new LandmarkPose(currentStateVectorSize, latestRobotPose,
                    lmark.obs[0], lmark.obs[1]);
            match.setId(lmark.id);
            currentStateVectorSize += match.getNumDimensions();
            nodes.add(match);
            // reset state
            this.chi2 = originalChi2;
            updateNodesPosition(originalStateVector);
            edges.remove(edges.size() - 1);
            assert LinAlg.equals(originalStateVector, this.getStateVector(), 0.000001);
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
    }*/

    class PossibleMatch implements Comparable<PossibleMatch>
    {
        public landmark_t lmark;
        public LandmarkPose pose;
        public double chi2;

        public PossibleMatch(landmark_t lmark, LandmarkPose pose) {
            this.lmark = lmark;
            this.pose = pose;
            chi2 = -1;
        }

        public int compareTo(PossibleMatch o)
        {
            return Double.compare(chi2, o.chi2);
        }
    }
}
