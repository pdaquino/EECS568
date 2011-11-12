package team;

import java.awt.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.vis.*;

public class FastSLAM implements Simulator.Listener {

    Config config;
    VisWorld vw;
    // Config vals
    double baseline;
    ArrayList<double[]> lmarkGroundTruth = new ArrayList<double[]>();
    // Particle management
    int NUM_PARTICLES;
    ArrayList<Particle> particles = new ArrayList<Particle>(NUM_PARTICLES);
    // Odometry info
    double[][] odomP;
    Random random = new Random(1748298757);

    public void init(Config config_, VisWorld vw_) {
        config = config_;
        vw = vw_;

        NUM_PARTICLES = config.requireInt("particles.numParticles");

        baseline = config.requireDouble("robot.baseline_m");
        odomP = new double[2][2];
        double[] sigLsigR = config.requireDoubles("noisemodels.odometryDiag");
        odomP[0][0] = sigLsigR[0] * sigLsigR[0];
        odomP[1][1] = sigLsigR[1] * sigLsigR[1];

        for (int i = 0;; i++) {
            double[] xy = config.getDoubles("landmarks.l" + i, null);
            if (xy == null) {
                break;
            }
            lmarkGroundTruth.add(xy);
        }

        // Init particles
        for (int i = 0; i < NUM_PARTICLES; i++) {
            particles.add(new Particle(config_, new double[3]));
        }
    }
    static int resampleCount = 1;

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets) {
        // Update particle positions by sampling around odom measurement
        for (Particle p : particles) {
            Matrix P = new Matrix(2, 2);
            P.set(0, 0, odomP[0][0] * odom.obs[0] * odom.obs[0]);
            P.set(1, 1, odomP[1][1] * odom.obs[1] * odom.obs[1]);
            MultiGaussian mg = new MultiGaussian(P.copyArray(), new double[2]);
            double[] dLdR = mg.sample(random);
            double chi2 = mg.chi2(dLdR);
            double logProb = -chi2/2;
            dLdR[0] = odom.obs[0] + dLdR[0];
            dLdR[1] = odom.obs[1] + dLdR[1];
            double[] local_xyt = new double[]{(dLdR[0] + dLdR[1]) / 2,
                0,
                Math.atan2((dLdR[1] - dLdR[0]), baseline)};
            
            p.updateLocation(local_xyt, chi2);
            p.addLogProb(logProb);
        }

        // Do feature matching (XXX for now, perfect). Specific to each
        // particle. (Data association)

        // no need to resample if there were no detections
        if (dets.size() > 0) {
            updateLandmarks(dets);
            resample();
        }
        assert particles.size() == NUM_PARTICLES;
        drawStuff(dets);
    }

    private void updateLandmark(Simulator.landmark_t det) {
        ArrayList<Simulator.landmark_t> detList = new ArrayList<Simulator.landmark_t>();
        detList.add(det);
        updateLandmarks(detList);
    }

    private void updateLandmarks(ArrayList<Simulator.landmark_t> detList) {
        for (Particle p : particles) {
            p.associateAndUpdateLandmarks(detList);
        }
    }

    private void resample() {
        /*double maxWeight = Double.NEGATIVE_INFINITY;
        for (Particle p: particles) {
        if (p.getWeight() > maxWeight) {
        maxWeight = p.getWeight();
        }
        }*/

        double weightTotal = 0;
        for (Particle p : particles) {
            weightTotal += p.getWeight();
        }
        //if(MathUtil.doubleEquals(weightTotal, 0)) {
        //throw new IllegalStateException("All weights are zero");
        //}
        ArrayList<Particle> newParticles = new ArrayList<Particle>(NUM_PARTICLES);
        for (int i = 0; i < NUM_PARTICLES; i++) {
            newParticles.add(sample(weightTotal));
        }
        particles = newParticles;
    }

    private Particle sample(double totalWeight) {
        double rand = random.nextDouble() * totalWeight;
        //int cnt = 0;
        double weightSum = 0;
        for (Particle p : particles) {
            weightSum += p.getWeight();
            if (weightSum >= rand) {
                return p.getSample();
            }
        }
        //System.err.printf("weigthSum = %f; rand = %f\n", weightSum, rand);
        throw new IllegalStateException("No particle could be sampled");

    }

    // find the rotation that best aligns the features in the map
    private double getAlignmentRotation(Particle p) {
        ArrayList<double[]> lmarks = p.getLandmarks();
        ArrayList<Integer> lmarkIds = p.getIDs();
        ArrayList<double[]> matchingTruthLmarks = new ArrayList<double[]>();
        for (int i = 0; i < lmarks.size(); i++) {
            matchingTruthLmarks.add(lmarkGroundTruth.get(lmarkIds.get(i)));
        }
        double alignRotation = RotationFit.fitRotation(p.getLandmarks(), matchingTruthLmarks);
        return alignRotation;
    }

    // Draw our own updates to screen
    void drawStuff(ArrayList<Simulator.landmark_t> dets) {
        // Pick the particle to render
        Particle best = null;
        double chi2 = Double.MAX_VALUE;
        double logProb = -Double.MAX_VALUE;
        for (Particle p : particles) {
            double temp = p.getChi2();
            if (chi2 > temp) {
                chi2 = temp;
                best = p;
            }
//            double temp = p.getLogProb();
//            if (logProb < temp) {
//                logProb = temp;
//                best = p;
//            }
            
        }
        System.out.println(best.getChi2() + "/" + best.getLogProb());
        double alignRotation = getAlignmentRotation(best);

        // Render the (best) robot
        {
            VisWorld.Buffer vb = vw.getBuffer("robot-local");
            vb.setDrawOrder(-100);
            VisWorld.Buffer vb2 = vw.getBuffer("robot-best");
            vb2.setDrawOrder(100);
            double[] xyt = best.getPose();
            double[] xyzrpy = new double[]{xyt[0], xyt[1], 0,
                0, 0, xyt[2]};
            vb2.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy),
                    new VisRobot(Color.yellow)));

//            System.out.printf("%d new observations\n", dets.size());
            VisRobot robot = new VisRobot(new Color(160, 30, 30));
            ArrayList<double[]> points = new ArrayList<double[]>();
            for (Particle p : particles) {
                xyt = p.getPose();
                xyzrpy = new double[]{xyt[0], xyt[1], 0,
                    0, 0, xyt[2]};
                //vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
                points.add(LinAlg.resize(xyt, 2));
            }
            vb.addBack(new VisPoints(new VisVertexData(points),
                    new VisConstantColor(new Color(160, 30, 30)),
                    1));
            vb.swap();
            vb2.swap();
        }

        // Render all landmark estimates
        {
            VisWorld.Buffer vb = vw.getBuffer("all-landmark-estimates");
            vb.setDrawOrder(-200);
            ArrayList<double[]> points = new ArrayList<double[]>();
            for (Particle p : particles) {
                points.addAll(p.getLandmarks());
            }
            vb.addBack(new VisPoints(new VisVertexData(points),
                    new VisConstantColor(Color.LIGHT_GRAY),
                    2));
            vb.swap();

        }
        // Draw the landmark observations
        {
            VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
            for (Simulator.landmark_t lmark : dets) {
                double[] obs = lmark.obs;
                ArrayList<double[]> obsPoints = new ArrayList<double[]>();
                obsPoints.add(LinAlg.resize(best.getPose(), 2));
                double rel_xy[] = {obs[0] * Math.cos(obs[1]), obs[0] * Math.sin(obs[1])};
                obsPoints.add(LinAlg.transform(best.getPose(), rel_xy));
                vb.addBack(new VisLines(new VisVertexData(obsPoints),
                        new VisConstantColor(lmark.id == -1 ? Color.gray : Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
            }
            vb.swap();
        }

        // Render (all) trajectories
        {
            VisWorld.Buffer vb = vw.getBuffer("particle-trajectories");
            vb.setDrawOrder(-100);
            VisWorld.Buffer vb2 = vw.getBuffer("best-trajectory");
            vb2.setDrawOrder(100);
            VisVertexData vvd = new VisVertexData(best.getTrajectory());
            VisConstantColor vccYellow = new VisConstantColor(Color.yellow);
            vb2.addBack(new VisLines(vvd, vccYellow, 1.5, VisLines.TYPE.LINE_STRIP));

            vb.swap();
            vb2.swap();
        }

        // Render the aligned trajectory
        {
            if (!Double.isNaN(alignRotation)) {
                VisWorld.Buffer vb = vw.getBuffer("aligned-trajectory");
                vb.setDrawOrder(200);
                ArrayList<double[]> alignedPoints = RotationFit.applyRotation(
                        alignRotation, best.getTrajectory());
                VisVertexData vvd = new VisVertexData(alignedPoints);
                vb.addBack(new VisLines(vvd, new VisConstantColor(Color.GREEN), 1.5, VisLines.TYPE.LINE_STRIP));

                vb.swap();
            }
        }

        // Render landmark estimates for best robot along with lines
        // connecting them to the landmark that spawned them
        ArrayList<double[]> lmarks = best.getLandmarks();
        ArrayList<Integer> lmarkIds = best.getIDs();
        {
            VisWorld.Buffer vb = vw.getBuffer("landmark-estimates");
            VisVertexData vvd = new VisVertexData(lmarks);
            VisConstantColor vccRed = new VisConstantColor(Color.red);
            vb.addBack(new VisPoints(vvd, vccRed, 5));

            vb.swap();
        }

        // Render aligned landmark estimates
        {
            if (!Double.isNaN(alignRotation)) {
                VisWorld.Buffer vb = vw.getBuffer("aligned-landmark-estimates");
                VisVertexData vvd = new VisVertexData(
                        RotationFit.applyRotation(alignRotation, lmarks));
                VisConstantColor vccOrange = new VisConstantColor(Color.ORANGE);
                vb.addBack(new VisPoints(vvd, vccOrange, 5));

                vb.swap();
            }
        }

        {
            VisWorld.Buffer vb = vw.getBuffer("landmark-groundTruth");
            ArrayList<double[]> lines = new ArrayList<double[]>();
            VisConstantColor vccCyan = new VisConstantColor(Color.cyan);
            for (int i = 0; i < lmarks.size(); i++) {
                if (lmarkIds.get(i) < 0) {
                    continue;
                }
                lines.add(lmarks.get(i));
                lines.add(lmarkGroundTruth.get(lmarkIds.get(i)));
            }
            VisVertexData vvd = new VisVertexData(lines);

            vb.addBack(new VisLines(vvd, vccCyan, 1, VisLines.TYPE.LINES));

            vb.swap();
        }

    }

    private void assertWeightsAreOne() {
        for (Particle p : particles) {
            assert (MathUtil.doubleEquals(p.getWeight(), 1.0));
        }
    }
}
