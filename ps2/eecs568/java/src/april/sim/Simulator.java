package april.sim;

import javax.swing.*;
import java.awt.*;

import java.util.*;

import april.vis.*;
import april.util.*;
import april.jmat.*;
import april.config.*;

public class Simulator implements Runnable
{
    public static interface Listener {
        public void init(Config c, VisWorld vw);

        public void update(odometry_t odom,
                           ArrayList<landmark_t> landmarks);
    }

    public static class landmark_t
    {
        public long utime;
        public int id;
        public double obs[];
    }

    public static class odometry_t
    {
        public long utime;
        public double obs[];
    }


    // Physical properties
    final double baseline; // 40 cm
    final double speed; //m/s


    // How long does each simulator tick simulate?
    private final double tickLength;

    // Noise models
    MultiGaussian landmarkMG;

    boolean knownDataAssoc;
    boolean loopSim;

    public VisWorld vw = new VisWorld();
    public VisLayer vl = new VisLayer(vw);
    public VisCanvas vc = new VisCanvas(vl);

    public JFrame jf = new JFrame("Simulator");
    public ParameterGUI pg = new ParameterGUI();

    // Controls the speed of the simulator
    Object tickWait = new Object();
    ArrayList<Listener> listeners = new ArrayList<Listener>();

    Config config;

    // Where's the robot at?
    private double truth_xyt[];
    private double local_xyt[]; //used for control


    // Where are the landmarks?
    private ArrayList<double[]> landmarks = new ArrayList<double[]>();

    // Where should the robot drive
    private ArrayList<double[]> waypoints = new ArrayList<double[]>();
    int waypointIdx = 0;

    // Where my random bits at?
    Random r = new Random(2);

    public Simulator(Config _config)
    {
        config = _config;

        baseline = config.requireDouble("robot.baseline_m");
        speed = config.requireDouble("robot.speed_ms");
        tickLength = config.requireDouble("simulator.tickLength_s");
        knownDataAssoc = config.requireBoolean("simulator.knownDataAssoc");
        loopSim = config.requireBoolean("simulator.loop");

        truth_xyt = config.requireDoubles("robot.start_xyt");
        local_xyt = new double[3];

        // Initialize robot position & landmarks
        for (int i = 0;; i++) {
            double[] li = config.getDoubles("landmarks.l"+i,null);
            if (li == null)
                break;
            landmarks.add(LinAlg.resize(li,3));
        }


        // Compute blind waypoints the robot will follow
        for (int i = 0;; i++) {
            double[] wpi = config.getDoubles("robot.waypoints.wp"+i,null);
            if (wpi ==null)
                break;
            waypoints.add(wpi);

        }


        // Initialize static noise models
        landmarkMG = new MultiGaussian(LinAlg.diag(config.requireDoubles("noisemodels.landmarkDiag")),
                                       config.getDoubles("noisemodels.landmarkMu",new double[2]));




        vl.cameraManager.fit2D(new double[]{-20,-20}, new double[]{20,20}, true);

        pg.addButtons("step","Step","start/stop","Start/Stop");
        StepThread steps = new StepThread();
        pg.addListener(steps);
        steps.start();

        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setSize(800,600);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setVisible(true);


        drawTruth(new ArrayList<landmark_t>(), new ArrayList<landmark_t>());
    }


    // Make sure to call this before running the simulator thread
    // otherwise race condition ensues
    public void addListener(Listener l)
    {
        listeners.add(l);
    }

    public void run()
    {
        int utime = 0;

        int loopCount = 0;
        while (true) {
            synchronized(tickWait) {
                try {
                    tickWait.wait();
                } catch(InterruptedException e){}
            }
            // 0. Check if we reached the current waypoint.
            if (LinAlg.distance(LinAlg.resize(local_xyt,2), waypoints.get(waypointIdx)) < .15) {
                waypointIdx = waypointIdx + 1;

                if (waypointIdx >= waypoints.size()) {
                    loopCount++;
                }
                if (!loopSim && loopCount > 0 && waypointIdx == 1)
                    break;

                waypointIdx = waypointIdx % waypoints.size();

            }


            // 1. preform one update of the simulator
            // A. Propagate robot motion
            //    -> compute odometery to reach the next waypoint
            double clean_odom[] = getDiffDrive(waypoints.get(waypointIdx), local_xyt); // in meters

            double odomD[] = config.requireDoubles("noisemodels.odometryDiag");
            double odomP[][] = {{odomD[0] * clean_odom[0], 0},
                                {0, odomD[1] *clean_odom[1]}};
            MultiGaussian odomMG = new MultiGaussian(odomP, new double[2]);


            double noisy_odom[] = LinAlg.add(clean_odom, odomMG.sample(r));

            double clean_odom_xyt[] = diffDriveToXyt(clean_odom);
            double noisy_odom_xyt[] = diffDriveToXyt(noisy_odom);

            double next_local[] = LinAlg.xytMultiply(local_xyt, clean_odom_xyt);
            double next_truth[] = LinAlg.xytMultiply(truth_xyt, noisy_odom_xyt);

            local_xyt = next_local;
            local_xyt[2] = MathUtil.mod2pi(local_xyt[2]);
            truth_xyt = next_truth;
            truth_xyt[2] = MathUtil.mod2pi(truth_xyt[2]);

            // B. Compute detections
            ArrayList<landmark_t> clean_dets = new ArrayList<landmark_t>();
            ArrayList<landmark_t> noisy_dets = new ArrayList<landmark_t>();
            // Generate observation of this landmark with some probability, if it's within view
            for (int i = 0; i < landmarks.size(); i++) {
                double l[] =  landmarks.get(i);
                double clean_det_xy[] = LinAlg.resize(LinAlg.xytInvMul31(truth_xyt,l),2);
                double clean_det_rt[] = {LinAlg.magnitude(clean_det_xy),
                                         Math.atan2(clean_det_xy[1],clean_det_xy[0])};

                double noisy_det_rt[] = LinAlg.add(clean_det_rt, landmarkMG.sample(r));


                // Accept a detection  ~mg.prob(x)
                // (Effectively a view angle check)

                if (Math.abs(noisy_det_rt[1]) < Math.PI/6 &&
                    noisy_det_rt[0] > .5 && noisy_det_rt[0] < 15.0 &&
                    r.nextDouble() < .15) {
                    landmark_t cleanObs = new landmark_t();
                    cleanObs.utime = utime;
                    cleanObs.obs = clean_det_rt;
                    cleanObs.id = (knownDataAssoc? i : -1);

                    landmark_t noisyObs = new landmark_t();
                    noisyObs.utime = utime;
                    noisyObs.obs = noisy_det_rt;
                    noisyObs.id = (knownDataAssoc? i : -1);

                    clean_dets.add(cleanObs);
                    noisy_dets.add(noisyObs);
               }
            }

            odometry_t cleanOdom = new odometry_t();
            cleanOdom.utime = utime;
            cleanOdom.obs = clean_odom;
            // 2. return detections
            for (Listener l : listeners)
                l.update(cleanOdom, noisy_dets);

            drawTruth(clean_dets, noisy_dets);


            utime += (long)(tickLength*1E6);
        }
    }



    private double[] getDiffDrive(double target_xy[], double robot_xyt[])
    {
        double wp[] =  target_xy; //waypoints.get(waypointIdx);
        double dx = wp[0] - robot_xyt[0];
        double dy = wp[1] - robot_xyt[1];
        double distXY = Math.sqrt(dx*dx +dy*dy);
        distXY = MathUtil.clamp(0,2.0, distXY);

        double angleToTarget = Math.atan2(dy, dx);
        double turn = MathUtil.mod2pi(angleToTarget - local_xyt[2]);
        double thetaErr = MathUtil.clamp(0.0, 1.0, Math.abs(turn));

        // Want to de-emphasize speed when near waypoint & have bad heading,
        // otherwise, we want to go fullspeed
        double speedScale =  thetaErr * distXY/2 + (1-thetaErr)*(1-thetaErr);

        double clean_odom[] = new double[]{speed*speedScale -turn*baseline/2,  speed*speedScale + turn*baseline/2};
        double norm = MathUtil.clamp(LinAlg.max(LinAlg.abs(clean_odom)),
                                     1.0, Double.MAX_VALUE);

        clean_odom = LinAlg.scale(clean_odom, speed*tickLength/norm);
        return clean_odom;
    }

    // input is {left_meters, right_meters}
    // output is {dx, dy, dtheta}
    private double[] diffDriveToXyt(double diff_drive[])
    {
        return new double[]{(diff_drive[0] + diff_drive[1]) /2, 0,
                            Math.atan((diff_drive[1] - diff_drive[0])/baseline)};

    }

    ArrayList<double[]> truthTrajectory = new ArrayList<double[]>();

    public void drawTruth(ArrayList<landmark_t> clean_dets, ArrayList<landmark_t> noisy_dets)
    {

        // Make a robot, save for later
        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        rpoints.add(new double[]{-.3, .3});
        rpoints.add(new double[]{-.3, -.3});
        rpoints.add(new double[]{.45,0});


        truthTrajectory.add(LinAlg.resize(truth_xyt,2));


        // Draw the robot
        {
            VisWorld.Buffer vb = vw.getBuffer("robot-truth");
            VisObject robot = new VisLines(new VisVertexData(rpoints),
                                           new VisConstantColor(Color.blue),
                                           3,
                                           VisLines.TYPE.LINE_LOOP);

            double xyzrpy[] = new double[]{truth_xyt[0], truth_xyt[1], 0,
                                           0, 0, truth_xyt[2]};
            vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
            vb.swap();
        }


        // Draw true Trajectory
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-truth");
            vb.addBack(new VisLines(new VisVertexData(truthTrajectory),
                                    new VisConstantColor(new Color(30,30,160)),
                                    1.5, VisLines.TYPE.LINE_STRIP));
            vb.swap();
        }





        // Draw the landmarks
        {
            ArrayList<double[]> star = new ArrayList<double[]>();
            int n = 5;
            double radskip = 2*(2*Math.PI/n);


            for (int i = 0; i < 2*n; i++) {
                double rad = i*radskip;
                double pt[] = {Math.cos(rad),Math.sin(rad)};
                star.add(pt);
            }

            VisVertexData vdat = new VisVertexData(star);

            VisWorld.Buffer vb = vw.getBuffer("landmarks");

            for (double l[] : landmarks) {
                vb.addBack(new VisChain(LinAlg.translate(l[0],l[1],0),
                                        LinAlg.scale(.25,.25,.25),
                                        new VisLines(vdat, new VisConstantColor(Color.green),
                                                     2, VisLines.TYPE.LINE_LOOP)));
            }
            vb.swap();
        }

        // Draw the waypoints
        {
            VisWorld.Buffer vb = vw.getBuffer("waypoints");
            vb.addBack(new VisPoints(new VisVertexData(waypoints),
                                     new VisConstantColor(Color.white),
                                     4));

            VisVertexData selDat = new VisVertexData();
            selDat.add(waypoints.get(waypointIdx));
            vb.addBack(new VisPoints(selDat,
                                     new VisConstantColor(Color.yellow),
                                     4));


            vb.swap();
        }
    }


    public void step()
    {
        synchronized(tickWait) {
            tickWait.notifyAll();
        }
    }

    class StepThread extends Thread implements ParameterListener
    {
        boolean running = config.requireBoolean("simulator.autoRun");

        public synchronized void parameterChanged(ParameterGUI pg, String name)
        {
            if (name.equals("step")) {
                step();
            }

            if (name.equals("start/stop")) {
                running = !running;
            }
        }

        public void run()
        {

            while(true) {
                if (running)
                    step();
                synchronized(this) {
                    try {
                        wait(100);
                    } catch(InterruptedException e){}
                }
            }

        }

    }


    public static void main(String args[]) throws java.io.IOException
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help");

        gopt.addString('c', "config", "", "Configuration file, e.g. ~/eecs568/config/sim.config");
        gopt.addString('l', "listener", "", "Class name of Simulator.Listener, e.g. april.sim.DummyListener");

        if (!gopt.parse(args) || gopt.getBoolean("help") || gopt.getExtraArgs().size() > 0) {
            gopt.doHelp();
            return;
        }


        Simulator sim  = new Simulator(new ConfigFile(gopt.getString("config")));


        Listener listener = (Listener)ReflectUtil.createObject(gopt.getString("listener"));
        if (listener == null) {
            System.out.println("Invalid listener classname: "+gopt.getString("listener"));
            System.exit(1);
        }

        listener.init(sim.config, sim.vw);
        sim.addListener(listener);

        new Thread(sim).start();
    }
}