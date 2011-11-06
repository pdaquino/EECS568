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

    // Particle management
    final int NUM_PARTICLES = 10000;
    ArrayList<Particle> particles = new ArrayList<Particle>(NUM_PARTICLES);

    // Odometry info
    double[][] odomP;

    public void init(Config config_, VisWorld vw_)
    {
        config = config_;
        vw = vw_;

        baseline = config.requireDouble("robot.baseline_m");
        odomP = new double[2][2];
        double[] sigLsigR = config.requireDoubles("noisemodels.odometryDiag");
        odomP[0][0] = sigLsigR[0];
        odomP[1][1] = sigLsigR[1];

        // Init particles
        for (int i = 0; i < NUM_PARTICLES; i++) {
            particles.add(new Particle(new double[3]));
        }
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets)
    {
        Random random = new Random();

        // Update particle positions by sampling around odom measurement
        //MultiGaussian mg = new MultiGaussian(odomP, odom.obs);
        MultiGaussian mg = new MultiGaussian(odomP, new double[2]);
        for (Particle p: particles) {
            double[] dLdR = mg.sample(random);
            dLdR[0] = odom.obs[0]*(1.0 + dLdR[0]);
            dLdR[1] = odom.obs[1]*(1.0 + dLdR[1]);
            double[] local_xyt = new double[] {(dLdR[0] + dLdR[1])/2,
                                                0,
                                                Math.atan((dLdR[1] - dLdR[0]) / baseline)};

            p.updateLocation(local_xyt);
        }

        // Do feature matching (XXX for now, perfect). Specific to each
        // particle. (Data association)
        double weightTotal = 0;
        for (Particle p: particles) {
            weightTotal += p.associateAndUpdateLandmarks(dets);
        }

        ArrayList<Particle> newParticles = new ArrayList<Particle>(NUM_PARTICLES);
        for (int i = 0; i < NUM_PARTICLES; i++) {
            double rand = random.nextDouble()*weightTotal;
            //System.out.printf("%f -- %f\n", rand, weightTotal);
            double weightSum = 0;
            int cnt = 0;
            for (Particle p: particles) {
                weightSum += p.getWeight();
                if (weightSum >= rand) {
                    newParticles.add(p.getSample());
                    break;
                }
                cnt++;
            }
            //System.out.printf("Chose particle %d\n", cnt);
        }
        particles = newParticles;

        drawStuff();
    }

    // Draw our own updates to screen
    void drawStuff()
    {
        // Render the robot
        {
            VisWorld.Buffer vb = vw.getBuffer("robot-local");
            for (Particle p: particles) {
                VisRobot robot = new VisRobot(new Color(160, 30, 30));
                double[] xyt = p.getPose();

                double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
                                               0, 0, xyt[2]};
                vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
            }
            vb.swap();
        }

        // Render all trajectories
        {
            VisWorld.Buffer vb = vw.getBuffer("particle-trajectories");
            for (Particle p: particles) {
                VisVertexData vvd = new VisVertexData(p.getTrajectory());
                VisConstantColor vccRed = new VisConstantColor(new Color(160, 30, 30));
                vb.addBack(new VisLines(vvd, vccRed, 1.5, VisLines.TYPE.LINE_STRIP));
            }
            vb.swap();
        }
    }
}
