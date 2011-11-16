package kinect;

import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

class KinectDemo
{
    Kinect kinect = new Kinect();

    RenderThread rt;
    KinectThread kt;

    public KinectDemo(GetOpt opts)
    {
        rt = new RenderThread(opts);
        rt.start();

        kt = new KinectThread();
        kt.start();
    }

    class KinectThread extends Thread
    {
        int fps = 60;
        boolean closeFlag;

        public KinectThread()
        {
            closeFlag = false;
        }

        public void run()
        {
            System.out.println("Starting kinect thread");
            kinect.init();
            kinect.start();

            Kinect.Frame f = null;
            while (!closeFlag) {
                f = kinect.getFrame();
                if (f != null) {}
                    rt.render(f);
                TimeUtil.sleep(1000/fps);
            }

            System.out.println("Buh-bye, now!");
            rt.allClear();
        }

        public void close()
        {
            closeFlag = true;
            kinect.stop();
            kinect.close();
        }
    }

    class RenderThread extends Thread
    {
        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;
        boolean clearToClose = false;

        Kinect.Frame currFrame = null;

        GetOpt opts;

        public RenderThread(GetOpt opts)
        {
            this.opts = opts;
            System.out.println("Starting render thread");
            JFrame jf = new JFrame("Kinect Demo");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.addWindowListener(new WindowAdapter() {
                synchronized public void windowClosing(WindowEvent e) {
                    System.out.println("Closing kinect...");
                    kt.close();
                    while (!clearToClose) {
                        try {
                            wait();
                        } catch (InterruptedException ex) {}
                    }
                }
            });
            jf.setLayout(new BorderLayout());
            jf.setSize(1280, 500);

            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);
            jf.add(vc, BorderLayout.CENTER);

            jf.setVisible(true);
        }

        synchronized public void allClear()
        {
            clearToClose = true;
            notifyAll();
        }

        synchronized public void render(Kinect.Frame frame)
        {
            currFrame = frame;
            notify();
        }

        synchronized public void run()
        {
            VisWorld.Buffer vbIm = vw.getBuffer("image");
            VisWorld.Buffer vbPts = vw.getBuffer("points");

            while (true) {
                if (currFrame != null && !opts.getBoolean("point-cloud")) {
                    BufferedImage rgb = makeRGB();
                    BufferedImage depth = makeDepth();

                    double[] xy0 = new double[2];
                    double[] xy1 = new double[] {currFrame.rgbWidth, currFrame.rgbHeight};
                    double[] xy2 = new double[] {currFrame.rgbWidth, 0};
                    double[] xy3 = new double[] {currFrame.rgbWidth+currFrame.depthWidth, currFrame.depthHeight};

                    double[][] rgbvert = new double[][] {{0,0,0},
                                                         {currFrame.rgbWidth,0,0},
                                                         {currFrame.rgbWidth,currFrame.rgbHeight,0},
                                                         {0,currFrame.rgbHeight,0}};
                    double[][] depthvert = new double[][] {{currFrame.rgbWidth,0,0},
                                                           {currFrame.rgbWidth+currFrame.depthWidth,0,0},
                                                           {currFrame.rgbWidth+currFrame.depthWidth,currFrame.depthHeight,0},
                                                           {currFrame.rgbWidth,currFrame.depthHeight,0}};
                    double[][] texcoords = new double[][] {{0,1},
                                                           {1,1},
                                                           {1,0},
                                                           {0,0}};

                    vbIm.addBack(new VisImage(new VisTexture(rgb, false),
                                              rgbvert,
                                              texcoords,
                                              Color.white));
                    vbIm.addBack(new VisImage(new VisTexture(depth, false),
                                              depthvert,
                                              texcoords,
                                              Color.white));

                    vbIm.swap();
                } else if (currFrame != null && opts.getBoolean("point-cloud")) {
                    ColorPointCloud pointCloud = new ColorPointCloud(currFrame);
                    VisVertexData vvd = new VisVertexData(pointCloud.points);
                    /*int[] colors = new int[pointCloud.numPoints()];
                    for (int i = 0; i < colors.length; i++) {
                        colors[i] = pointCloud.colors.get(i);
                    }
                    VisColorData vcd = new VisColorData(colors);*/
                    //VisConstantColor vcd = new VisConstantColor(Color.white);

                    vbPts.addBack(new VisPoints(vvd, pointCloud.vcd, 1));

                    vbPts.swap();
                }

                try {
                    wait();
                } catch (InterruptedException ex) {}
            }
        }

        BufferedImage makeRGB()
        {
            assert (currFrame != null);
            assert (currFrame.argb.length == currFrame.rgbHeight*currFrame.rgbWidth);
            BufferedImage im = new BufferedImage(currFrame.rgbWidth, currFrame.rgbHeight, BufferedImage.TYPE_INT_RGB);
            int[] buf = ((DataBufferInt)(im.getRaster().getDataBuffer())).getData();
            for (int i = 0; i < buf.length; i++) {
                buf[i] = currFrame.argb[i];
            }

            return im;
        }

        BufferedImage makeDepth()
        {
            assert (currFrame != null);
            assert (currFrame.depth.length == currFrame.depthHeight*currFrame.depthWidth);
            BufferedImage im = new BufferedImage(currFrame.depthWidth, currFrame.depthHeight, BufferedImage.TYPE_INT_RGB);
            int[] buf = ((DataBufferInt)(im.getRaster().getDataBuffer())).getData();
            double[] cutoffs = new double[] {1.0, 1.75, 2.5, 3.25, 4.0, 5.0};
            for (int i = 0; i < buf.length; i++) {
                // XXX Improved color mapping. Optimal range is ~0.8m - 3.5m
                // white -> close
                // red
                // orange
                // yellow
                // green
                // blue
                // magenta
                // black -> bad values
                double m = currFrame.depthToMeters(currFrame.depth[i]);
                if (m < 0) {
                    buf[i] = 0;
                    continue;
                }
                int r,g,b;
                if (m < cutoffs[0]) {
                    r = 0xff;
                    g = 0xff - (int) (0xff * m/cutoffs[0]);
                    b = 0xff - (int) (0xff * m/cutoffs[0]);
                } else if (m < cutoffs[1]) {
                    r = 0xff;
                    g = 0xff - (int) (0xff * ((cutoffs[1] - m)/(cutoffs[1]-cutoffs[0])));
                    b = 0;
                } else if (m < cutoffs[2]) {
                    r = (int) (0xff * ((cutoffs[2] - m)/(cutoffs[2]-cutoffs[1])));
                    g = 0xff;
                    b = 0;
                } else if (m < cutoffs[3]) {
                    r = 0;
                    g = (int) (0xff * ((cutoffs[3] - m)/(cutoffs[3]-cutoffs[2])));
                    b = 0xff - (int) (0xff * ((cutoffs[3] - m)/(cutoffs[3]-cutoffs[2])));
                } else if (m < cutoffs[4]) {
                    r = 0xff - (int) (0xff * ((cutoffs[4] - m)/(cutoffs[4]-cutoffs[3])));
                    g = 0;
                    b = 0xff;
                } else if (m < cutoffs[5]) {
                    r = (int) (0xff * ((cutoffs[5] - m)/(cutoffs[5]-cutoffs[4])));
                    g = 0;
                    b = (int) (0xff * ((cutoffs[5] - m)/(cutoffs[5]-cutoffs[4])));
                } else {
                    r = 0;
                    g = 0;
                    b = 0;
                }

                buf[i] = 0xff000000 | (r << 16) | (g << 8) | b;
            }

            return im;
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean((char)0,"point-cloud",false,"Render colored point cloud");
        opts.addBoolean('h', "help", false, "Show this help screen");

        if (!opts.parse(args)) {
            System.err.println("ERR: Option failure = "+opts.getReason());
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
        }

        KinectDemo kd = new KinectDemo(opts);
    }
}
