package rgbdslam.test;

import java.util.*;
import java.awt.*;
import java.awt.event.*;
import javax.swing.*;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;

import kinect.*;
import rgbdslam.VoxelArray;

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
        int fps = 15;
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
        Kinect.Frame renderFrame = null;

        // Vis
        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;

        // Flying around stuff
        double xticks, yticks, zticks, tticks;
        int dir;
        double vel = 1.0; // m/s
        double theta_vel = Math.toRadians(15);  // rad/sec
        int fps = 15;

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
                        renderFrame = lastFrame;
                    }
                }
            });

            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);

            DefaultCameraManager dcm = new DefaultCameraManager();
            dcm.UI_ANIMATE_MS = 50;
            vl.cameraManager = dcm;
            vl.cameraManager.setDefaultPosition(new double[] {0, 0, 5}, new double[] {0, 0, 0}, new double[] {0, 1, 0});
            vl.cameraManager.uiDefault();
            vl.addEventHandler(new MyEventAdapter());

            /*{
                VisWorld.Buffer vb = vw.getBuffer("grid");
                vb.addBack(new VzGrid(new VzLines.Style(Color.gray, 1)));
                vb.swap();
            }*/

            jf.add(vc, BorderLayout.CENTER);
            jf.add(pg, BorderLayout.SOUTH);

            jf.setVisible(true);
        }

        synchronized public void toggleDirection(int vk)
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

        synchronized public void updateVoxelRes(double res)
        {
            va = new VoxelArray(res); // Throws away old data
        }

        synchronized public void renderFrame(Kinect.Frame f)
        {
            assert (f != null);
            lastFrame = f;
            //notify();
        }

        private double[] getCameraTranslation(double dt)
        {

            return new double[] {xticks*vel*dt, yticks*vel*dt, zticks*vel*dt, 0, 0, tticks*theta_vel*dt};
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

                if (renderFrame != null) {
                    VisWorld.Buffer vb = vw.getBuffer("voxels");

                    ColorPointCloud cpc = new ColorPointCloud(renderFrame);
                    va.voxelizePointCloud(cpc, i4);


                    /*ArrayList<VisChain> voxels = va.getBoxes();
                    //System.out.println(voxels.size());

                    for (VisChain vc: voxels) {
                        vb.addBack(new VisLighting(false, vc));
                    }*/
                    vb.addBack(va.getPointCloud());

                    vb.swap();
                }

                try {
                    wait(1000/fps);
                } catch (InterruptedException ex) {}
            }
        }

        class MyEventAdapter extends VisEventAdapter
        {
            // Deal with repeated key presses
            public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
            {
                int vk = e.getKeyCode();
                toggleDirection(vk);

                return true;
            }

            public boolean keyTyped(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
            {
                int vk = e.getKeyCode();

                return false;
            }

            public boolean keyReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
            {
                int vk = e.getKeyCode();
                //toggleDirection(vk, false);

                return true;
            }
        }
    }

    static public void main(String[] args)
    {
        VoxelTest vt = new VoxelTest();
    }

}
