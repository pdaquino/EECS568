package rgbdslam;

import java.util.*;

import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;

import kinect.*;

import rgbdslam.*;

public class RGBDSLAM
{
    KinectThread kt;
    RenderThread rt;
    RGBDThread rgbd;

    double DEFAULT_RES = 0.05;
    double currRes = DEFAULT_RES;
    VoxelArray globalVoxelFrame = new VoxelArray(DEFAULT_RES);

    // State
    boolean loadedFromFile = false;

    public RGBDSLAM(GetOpt opts)
    {
        // Load from file
        System.out.println(opts.getString("file"));
        if (opts.getString("file") != null) {
            System.out.println("Loading from file...");
            VoxelArray va = VoxelArray.readFromFile(opts.getString("file"));
            if (va != null) {
                globalVoxelFrame = va;
                loadedFromFile = true;
                System.out.println("Successfully loaded!");
            }
        }

        kt = new KinectThread();
        rt = new RenderThread();
        rgbd = new RGBDThread();

        if (!loadedFromFile) {
            kt.start();
        }
        rt.start();
        if (!loadedFromFile) {
            rgbd.start();
        }
    }

    class KinectThread extends Thread
    {
        int pollRate = 5;

        boolean closeSignal = false;
        boolean closed = true;

        public void run()
        {
            closed = false;
            Kinect kinect = new Kinect();

            kinect.init();
            kinect.start();

            while (true) {
                Kinect.Frame f = kinect.getFrame();
                if (f != null) {
                    // XXX Might want to dump frames
                    rgbd.handleFrame(f);
                }

                if (closeSignal)
                    break;

                TimeUtil.sleep(1000/pollRate);
            }

            System.out.println("Stopping kinect...");
            kinect.stop();
            System.out.println("Kinect stopped");
            System.out.println("Closing kinect...");
            kinect.close();
            System.out.println("Kienct closed");

            closed = true;
        }

        synchronized public void close()
        {
            closeSignal = true;
        }

        synchronized public boolean isClosed()
        {
            return closed;
        }

        synchronized public void setPollRate(int pr)
        {
            pollRate = pr;
        }
    }

    class RenderThread extends Thread
    {
        int fps = 5;

        // Vis
        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;

        // Knobs
        ParameterGUI pg;

        // Flying around stuff
        double xticks, yticks, zticks, tticks;
        double vel = 1.0; // m/s
        double theta_vel = Math.toRadians(15);  // rad/sec

        public RenderThread()
        {
            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);

            JFrame jf = new JFrame("RGBDSLAM Visualization");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setSize(1280, 720);
            jf.setLayout(new BorderLayout());

            // Make sure kinect gets closed
            jf.addWindowListener(new WindowAdapter() {
                synchronized public void windowClosing(WindowEvent e) {
                    System.out.println("Sending kinect close message...");
                    kt.close();
                    while (!kt.isClosed()) {
                        try {
                            System.out.println("Kinect not yet closed...");
                            wait(1000);
                        } catch (InterruptedException ex) {}
                    }
                    System.out.println("Shutdown complete");
                    System.out.println("Goodbye!");
                }
            });

            pg = new ParameterGUI();
            pg.addDoubleSlider("resolution", "Voxel Resolution (m)", 0.005, 0.5, DEFAULT_RES);
            pg.addIntSlider("kfps", "Kinect FPS", 1, 30, 5);
            pg.addIntSlider("rfps", "Render FPS", 1, 60, 15);
            pg.addButtons("save", "Save to file");
            pg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name) {
                    if (name.equals("resolution") && !loadedFromFile) {
                        //updateVoxelRes(pg.gd("resolution"));
                    } else if (name.equals("save")) {
                        long time = System.currentTimeMillis();
                        synchronized (globalVoxelFrame) {
                            String filename = "va_"+time+".vx";
                            globalVoxelFrame.writeToFile(filename);
                            System.out.println("Saved to "+filename);
                        }
                    } else {
                        updateFPS();
                    }
                }
            });
            updateFPS();

            VzGrid.addGrid(vw);


            // XXX May need to change default position. Moving still
            // glitchy. Twitches and doesn't respect our axes
            /*DefaultCameraManager dcm = new DefaultCameraManager();
            dcm.UI_ANIMATE_MS = 25;
            vl.cameraManager = dcm;
            vl.cameraManager.setDefaultPosition(new double[] {0, 0, 5}, new double[] {0, 0, 0}, new double[] {0, 1, 0});
            vl.cameraManager.uiDefault();*/
            vl.addEventHandler(new MyEventAdapter());

            jf.add(vc, BorderLayout.CENTER);
            jf.add(pg, BorderLayout.SOUTH);

            jf.setVisible(true);
        }

        synchronized public void run()
        {
            Tic tic = new Tic();
            while (true) {
                // XXX Some bugs
                double[] xyzrpy = getCameraXYZRPY(tic.toctic());
                if (vc.getLastRenderInfo() != null) {
                    VisCameraManager.CameraPosition cpos = vc.getLastRenderInfo().cameraPositions.get(vl);

                    double[] yaxis = LinAlg.normalize(cpos.up);
                    double[] nzaxis = LinAlg.normalize(LinAlg.subtract(cpos.lookat, cpos.eye));
                    double[] xaxis = LinAlg.normalize(LinAlg.crossProduct(nzaxis, yaxis));

                    double[][] rotation = LinAlg.quatToMatrix(LinAlg.angleAxisToQuat(xyzrpy[5], yaxis));

                    // Translation
                    double[] eye = LinAlg.copy(cpos.eye);
                    double[] lookat = LinAlg.copy(cpos.lookat);
                    double x = xyzrpy[0];
                    double y = xyzrpy[1];
                    double z = xyzrpy[2];
                    double[] dx = new double[] {x*xaxis[0], x*xaxis[1], x*xaxis[2]};
                    double[] dy = new double[] {y*yaxis[0], y*yaxis[1], y*yaxis[2]};
                    double[] dz = new double[] {z*nzaxis[0], z*nzaxis[1], z*nzaxis[2]};
                    eye = LinAlg.add(dx, LinAlg.add(dy, LinAlg.add(dz, eye)));
                    lookat = LinAlg.add(dx, LinAlg.add(dy, LinAlg.add(dz, lookat)));

                    // Rotation
                    double[][] eye_trans = LinAlg.translate(eye);
                    double[][] eye_inv = LinAlg.inverse(eye_trans);
                    LinAlg.timesEquals(eye_trans, rotation);
                    LinAlg.timesEquals(eye_trans, eye_inv);
                    lookat = LinAlg.transform(eye_trans, lookat);

                    vl.cameraManager.uiLookAt(eye, lookat, cpos.up, false);
                }

                synchronized (globalVoxelFrame) {
                    if (globalVoxelFrame.size() > 0) {
                        VisWorld.Buffer vb = vw.getBuffer("voxels");
                        vb.addBack(new VisLighting(false,
                                                   globalVoxelFrame.getPointCloud()));
                        vb.swap();
                    }
                }

                TimeUtil.sleep(1000/fps);
            }
        }

        public void updateFPS()
        {
            fps = pg.gi("rfps");
            kt.setPollRate(pg.gi("kfps"));
        }

        private double[] getCameraXYZRPY(double dt)
        {

            return new double[] {xticks*vel*dt, yticks*vel*dt, zticks*vel*dt, 0, 0, tticks*theta_vel*dt};
        }

        public void toggleDirection(int vk)
        {
            if (vk == KeyEvent.VK_DOWN) {
                yticks -= 1;
            }
            if (vk == KeyEvent.VK_UP) {
                yticks += 1;
            }

            if (vk == KeyEvent.VK_W) {
                zticks += 1;
            }
            if (vk == KeyEvent.VK_S) {
                zticks -= 1;
            }

            if (vk == KeyEvent.VK_D) {
                xticks += 1;
            }
            if (vk == KeyEvent.VK_A) {
                xticks -= 1;
            }

            if (vk == KeyEvent.VK_LEFT) {
                tticks += 1;
            }
            if (vk == KeyEvent.VK_RIGHT) {
                tticks -= 1;
            }
            System.out.printf("[x:%f] [y:%f] [z:%f] [t:%f]\n", xticks, yticks, zticks, tticks);

        }


    }

    class RGBDThread extends Thread
    {
        Kinect.Frame currFrame = null;
        Kinect.Frame lastFrame = null;

        synchronized public void run()
        {
            Matrix rbt = Matrix.identity(4,4);

            while (true) {
                if (currFrame != null && lastFrame != null) {
                    // Deal with new data
                    // XXX Temporary
                    ColorPointCloud cpc = new ColorPointCloud(currFrame);
                    VoxelArray va = new VoxelArray(DEFAULT_RES); // XXX

                    // Extract features, perform RANSAC and ICP // XXX No ICP yet
                    //Matrix transform = getTransform(cpc);
                    //transform.print();
                    //rbt = rbt.times(transform);

                    ICP icp = new ICP(new ColorPointCloud(lastFrame, 10));
                    double[][] transform = icp.match(new ColorPointCloud(currFrame, 10), Matrix.identity(4,4).copyArray());
                    //cpc.rbt(transform);
                    rbt = rbt.times(new Matrix(transform));
                    /*for(int i=0; i<rbt.length; i++){
                        for (int j=0; j[0].length; j++){
                            System.out.print(transform[i][j]+"\t");
                        }
                        System.out.println();
                        }*/

                    va.voxelizePointCloud(cpc);

                    // Let render thread do its thing
                    synchronized (globalVoxelFrame) {
                        globalVoxelFrame.merge(va, rbt);
                    }
                }

                try {
                    wait();
                } catch (InterruptedException ex) {}
            }
        }

        synchronized public void handleFrame(Kinect.Frame frame)
        {
            lastFrame = currFrame;
            currFrame = frame;
            notifyAll();
        }

        private Matrix getTransform(ColorPointCloud cpc)
        {
            int[] argbC = currFrame.argb;
            int[] argbL = lastFrame.argb;
            ArrayList<ImageFeature> featuresC = OpenCV.extractFeatures(argbC, 640);
            ArrayList<ImageFeature> featuresL = OpenCV.extractFeatures(argbL, 640);

            // Set xyz (world) coordinates
            for(ImageFeature fc: featuresC){
                fc.setXyz(cpc.Project(LinAlg.copyDoubles(fc.xy())));
            }
            for(ImageFeature fl: featuresL){
                fl.setXyz(cpc.Project(LinAlg.copyDoubles(fl.xy())));
            }

            // Match SIFT features -> RANSAC
            DescriptorMatcher dm = new DescriptorMatcher(featuresL, featuresC);
            ArrayList<DescriptorMatcher.Match> matches = dm.match();
            ArrayList<DescriptorMatcher.Match> inliers = new ArrayList<DescriptorMatcher.Match>();

            return new Matrix(RANSAC.RANSAC(matches, inliers));
        }
    }


    class MyEventAdapter extends VisEventAdapter
    {
        // Deal with repeated key presses
        public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
        {
            int vk = e.getKeyCode();
            rt.toggleDirection(vk);

            return false;
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('f', "file", null, "Load scene from file");

        if (!opts.parse(args)) {
            System.err.println("ERR: Opts error - " + opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        RGBDSLAM rgbdSLAM = new RGBDSLAM(opts);
    }
}
