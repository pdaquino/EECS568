package rgbdslam;

import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;

import kinect.*;

public class VoxelTest
{
    Object frameLock = new Object();

    KinectThread kt;
    RenderThread rt;

    public VoxelTest()
    {
        kt = new KinectThread();
        rt = new RenderThread();
        kt.start();
        rt.start();
    }

    class KinectThread extends Thread
    {
        int fps = 60;
        boolean closeSignal = false;
        boolean closed = false;

        public void run()
        {
            Kinect kinect = new Kinect();

            kinect.init();
            kinect.start();

            while (true) {
                Kinect.Frame f = kinect.getFrame();
                if (f != null) {
                    rt.renderFrame(f);
                }
                if (closeSignal())
                    break;
                TimeUtil.sleep(1000/fps);
            }

            System.out.println("Kinect stopping...");
            kinect.stop();
            System.out.println("Kinect stopped");
            kinect.close();

            closed = true;
            System.out.println("Kinect closed");
        }

        synchronized public void close()
        {
            System.out.println("kinect.close()");
            closeSignal = true;
        }

        synchronized public boolean isClosed()
        {
            return closed;
        }

        synchronized public boolean closeSignal()
        {
            return closeSignal;
        }
    }

    class RenderThread extends Thread
    {
        // Kinect Thread
        Kinect.Frame lastFrame = null;

        // Vis
        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;

        int dir;
        double vel = 5.0; // m/s
        double theta_vel = Math.toRadians(30);  // rad/sec
        int fps = 30;

        final int UP = 1;
        final int DOWN = 2;
        final int LEFT = 4;
        final int RIGHT = 8;
        final int FORWARD = 16;
        final int BACK = 32;
        final int TURN_R = 64;
        final int TURN_L = 128;

        // Knobs
        ParameterGUI pg = new ParameterGUI();

        // Voxels
        double defaultRes = 0.1;
        VoxelArray va = new VoxelArray(defaultRes);

        public RenderThread()
        {

            JFrame jf = new JFrame("Voxel test");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setSize(1200, 720);
            jf.setLayout(new BorderLayout());

            jf.addWindowListener(new WindowAdapter() {
                synchronized public void windowClosing(WindowEvent e) {
                    System.out.println("Closing kinect...");
                    kt.close();
                    while (!kt.isClosed()) {
                        try {
                            System.out.println("Kinect not yet closed...");
                            wait(1000);
                        } catch (InterruptedException ex) {}
                    }
                    System.out.println("Goodbye!");
                }
            });

            pg.addDoubleSlider("res", "Voxel Resolution (m)", 0.01, 0.5, defaultRes);
            pg.addButtons("reset", "Reset Voxel Cloud");
            pg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name) {
                    if (name.equals("res")) {
                        updateVoxelRes(pg.gd("res"));
                    } else if (name.equals("reset")) {
                        updateVoxelRes(pg.gd("res"));
                    }
                }
            });

            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);

            DefaultCameraManager dcm = new DefaultCameraManager();
            dcm.UI_ANIMATE_MS = 0;
            vl.cameraManager = dcm;
            vl.cameraManager.setDefaultPosition(new double[] {-5, 0, 0}, new double[] {5, 0, 0}, new double[] {0, 0, 1});
            vl.cameraManager.uiDefault();
            vl.addEventHandler(new MyEventAdapter());

            {
                VisWorld.Buffer vb = vw.getBuffer("grid");
                vb.addBack(new VzGrid(new VzLines.Style(Color.gray, 1)));
                vb.swap();
            }

            jf.add(vc, BorderLayout.CENTER);
            jf.add(pg, BorderLayout.SOUTH);

            jf.setVisible(true);
        }

        synchronized public void toggleDirection(int vk, boolean press)
        {
            switch (vk) {
                case KeyEvent.VK_UP:
                    if (press)
                        dir |= UP;
                    else
                        dir &= ~UP;
                    break;
                case KeyEvent.VK_DOWN:
                    if (press)
                        dir |= DOWN;
                    else
                        dir &= ~DOWN;
                    break;
                case KeyEvent.VK_LEFT:
                    if (press)
                        dir |= TURN_L;
                    else
                        dir &= ~TURN_L;
                    break;
                case KeyEvent.VK_RIGHT:
                    if (press)
                        dir |= TURN_R;
                    else
                        dir &= ~TURN_R;
                    break;
                case KeyEvent.VK_A:
                    if (press)
                        dir |= LEFT;
                    else
                        dir &= ~LEFT;
                    break;
                case KeyEvent.VK_D:
                    if (press)
                        dir |= RIGHT;
                    else
                        dir &= ~RIGHT;
                    break;
                case KeyEvent.VK_W:
                    if (press)
                        dir |= FORWARD;
                    else
                        dir &= ~FORWARD;
                    break;
                case KeyEvent.VK_S:
                    if (press)
                        dir |= BACK;
                    else
                        dir &= ~BACK;
                    break;
            }
            //System.out.println(Integer.toBinaryString(dir));
        }

        synchronized public void updateVoxelRes(double res)
        {
            va = new VoxelArray(res); // Throws away old data
        }

        synchronized public void renderFrame(Kinect.Frame f)
        {
            assert (f != null);
            lastFrame = f;
            notify();
        }

        private double[] getCameraTranslation(double dt)
        {
            double x = 0, y = 0, z = 0, t = 0;
            if ((dir & UP) > 0) {
                y -= 1;
            }
            if ((dir & DOWN) > 0) {
                y += 1;
            }

            if ((dir & FORWARD) > 0) {
                z += 1;
            }
            if ((dir & BACK) > 0) {
                z -= 1;
            }

            if ((dir & RIGHT) > 0) {
                x += 1;
            }
            if ((dir & LEFT) > 0) {
                x -= 1;
            }

            // Add turning XXX
            if ((dir & TURN_L) > 0) {
                t += 1;
            }
            if ((dir & TURN_R) > 0) {
                t -= 1;
            }

            return new double[] {x*vel*dt, y*vel*dt, z*vel*dt, 0, 0, t*theta_vel*dt};
        }

        synchronized public void run()
        {
            Matrix i4 = Matrix.identity(4,4);

            Tic tic = new Tic();
            while (true) {
                // Move camera
                double[] xyzrpy = getCameraTranslation(tic.toctic());
                // XXX Note: this will make normal vis controls wacky
                if (vc.getLastRenderInfo() != null) {
                    VisCameraManager.CameraPosition cpos = vc.getLastRenderInfo().cameraPositions.get(vl);

                    double[] rpy = new double[] {xyzrpy[3], xyzrpy[4], xyzrpy[5]};

                    vl.cameraManager.uiRotate(LinAlg.rollPitchYawToQuat(rpy));

                    double[] eye, lookat;
                    eye = LinAlg.copy(cpos.eye);
                    lookat = LinAlg.copy(cpos.lookat);

                    double[] vec = LinAlg.normalize(LinAlg.subtract(lookat, eye));

                    // XXX This part totally doesn't work worth shit
                    /*double[] q = LinAlg.quatCompute(new double[] {0, 0, -1}, vec);

                    // Translate camera in coordinates relative to its current orientation
                    double[][] cam_trans = LinAlg.translate(LinAlg.resize(xyzrpy, 3));
                    double[][] cam_xform = LinAlg.quatToMatrix(q);

                    LinAlg.timesEquals(cam_trans, cam_xform);

                    eye = LinAlg.transform(cam_trans, eye);
                    lookat = LinAlg.transform(cam_trans, lookat);
                    vl.cameraManager.uiLookAt(eye, lookat, cpos.up, false);*/

                }

                if (lastFrame != null) {
                    VisWorld.Buffer vb = vw.getBuffer("voxels");

                    ColorPointCloud cpc = new ColorPointCloud(lastFrame);
                    va.voxelizePointCloud(cpc, i4);

                    ArrayList<VisChain> voxels = va.getBoxes();
                    //System.out.println(voxels.size());

                    for (VisChain vc: voxels) {
                        vb.addBack(vc);
                    }

                    vb.swap();
                }

                try {
                    wait(1000/fps);
                } catch (InterruptedException ex) {}
            }
        }

        class MyEventAdapter extends VisEventAdapter
        {
            // XXX Key events are broken
            public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
            {
                int vk = e.getKeyCode();
                toggleDirection(vk, true);

                return true;
            }

            public boolean keyReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
            {
                int vk = e.getKeyCode();
                toggleDirection(vk, false);

                return true;
            }
        }
    }

    static public void main(String[] args)
    {
        VoxelTest vt = new VoxelTest();
    }

}
