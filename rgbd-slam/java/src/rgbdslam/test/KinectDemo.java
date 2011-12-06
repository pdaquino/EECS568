package rgbdslam.test;

import rgbdslam.*;

import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;
import kinect.*;
import rgbdslam.DescriptorMatcher.Match;

class KinectDemo {

    Kinect kinect = new Kinect();
    RenderThread rt;
    KinectThread kt;
    static final int WIDTH = Constants.WIDTH;
    static final int HEIGHT = Constants.HEIGHT;
    double[] translateH = new double[]{WIDTH, 0, 0};
    double[] translateV = new double[]{0, -HEIGHT, 0};

    public KinectDemo(GetOpt opts) {
        rt = new RenderThread(opts);
        rt.start();

        kt = new KinectThread();
        kt.start();
    }

    class KinectThread extends Thread {

        int fps = 15;
        int rate = 240; // capture rate for images
        boolean closeFlag;

        public KinectThread() {
            closeFlag = false;
        }

        public void run() {
            System.out.println("Starting kinect thread");
            kinect.init();
            kinect.start();

            int counter = 0; // for taking picture files every x frames

            Kinect.Frame f = null;
            while (!closeFlag) {
                f = kinect.getFrame();

                if ((counter % rate == 0) && (f != null)) {
                    //kinect.saveRGB(f);
                }

                if (f != null) {
                }
                rt.render(f);
                TimeUtil.sleep(1000 / fps);
            }

            System.out.println("Buh-bye, now!");
            rt.allClear();
        }

        public void close() {
            closeFlag = true;
            kinect.stop();
            kinect.close();
        }
    }

    class RenderThread extends Thread {

        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;
        boolean clearToClose = false;
        Kinect.Frame currFrame = null;
        Kinect.Frame lastFrame = null;
        GetOpt opts;

        public RenderThread(GetOpt opts) {

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
                        } catch (InterruptedException ex) {
                        }
                    }
                }
            });
            jf.setLayout(new BorderLayout());
            jf.setSize(1280, 500);


            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);
            //vl.cameraManager.setDefaultPosition(new double[] {10.0, 5.0, 5.0},               // Camera position in xyz
            //new double[] {0.0, 0.0, 0.0},               // Point the camera is looking at in xyz
            //                            new double[] {-1.0, -1.0, 1.0});


            jf.add(vc, BorderLayout.CENTER);

            jf.setVisible(true);
        }

        synchronized public void allClear() {
            clearToClose = true;
            notifyAll();
        }

        synchronized public void render(Kinect.Frame frame) {
            lastFrame = currFrame;
            currFrame = frame;
            notify();
        }

        synchronized public void run() {
            VisWorld.Buffer vbIm = vw.getBuffer("image");
            VisWorld.Buffer vbPts = vw.getBuffer("points");

            while (true) {
                if (currFrame != null && lastFrame != null && opts.getBoolean("alignment")) {
                    BufferedImage rgbC = currFrame.makeRGB();
                    BufferedImage depthC = currFrame.makeDepth();
                    BufferedImage rgbL = currFrame.makeRGB();
                    BufferedImage depthL = currFrame.makeDepth();
                    
                    ColorPointCloud currPtCloud = new ColorPointCloud(currFrame);
                    ColorPointCloud lastPtCloud = new ColorPointCloud(lastFrame);

                    double[] xy0 = new double[2];
                    double[] xy1 = new double[]{WIDTH, HEIGHT};
                    double[] xy2 = new double[]{WIDTH, 0};
                    double[] xy3 = new double[]{2 * WIDTH, HEIGHT};

                    // Get each frame's features
                    int[] argbC = currFrame.argb;
                    int[] argbL = lastFrame.argb;
                    ArrayList<ImageFeature> featuresC = OpenCV.extractFeatures(argbC, 640);
                    ArrayList<ImageFeature> featuresL = OpenCV.extractFeatures(argbL, 640);

                    // Set xyz (world) coordinates
                    for (ImageFeature fc : featuresC) {
                        fc.setXyz(currPtCloud.Project(LinAlg.copyDoubles(fc.xy())));
                    }
                    for (ImageFeature fl : featuresL) {
                        fl.setXyz(lastPtCloud.Project(LinAlg.copyDoubles(fl.xy())));
                    }

                    // Match SIFT features -> RANSAC
                    DescriptorMatcher dm = new DescriptorMatcher(featuresL, featuresC);
                    ArrayList<DescriptorMatcher.Match> matches = dm.match();
                    ArrayList<DescriptorMatcher.Match> inliers = new ArrayList<DescriptorMatcher.Match>();
                    double[][] transform = RANSAC.RANSAC(matches, inliers);
                    for (int i = 0; i < transform.length; i++) {
                        for (int j = 0; j < transform[0].length; j++) {
                            System.out.print(transform[i][j] + "\t");
                        }
                        System.out.println();
                    }
                    //LinAlg.print(transform);
                    plotFeatures(featuresC, rgbC, featuresL, rgbL);

                    VzLines lines = plotCorrespondences(inliers);

                    vbIm.addBack(new VzImage(rgbC, VzImage.FLIP));
                    vbIm.addBack(new VisChain(LinAlg.translate(translateH),
                            new VzImage(rgbL, VzImage.FLIP)));
                    vbIm.addBack(new VisChain(LinAlg.scale(1, -1, 1), LinAlg.translate(translateV), lines));

                    vbIm.swap();
                }
                try {
                    wait();
                } catch (InterruptedException ex) {  }
            }
        }

        protected VzLines plotCorrespondences(ArrayList<Match> inliers) {
            // Lines between corresponding features
            ArrayList<double[]> correspondences = new ArrayList<double[]>();
            for (DescriptorMatcher.Match m : inliers) {
                double[] f1 = LinAlg.copyDoubles(m.feature1.xy());
                double[] f2 = LinAlg.copyDoubles(m.feature2.xy());
                f1 = LinAlg.transform(translateH, f1);

                correspondences.add(f1);
                correspondences.add(f2);
            }
            VisColorData vcd = new VisColorData();
            for (int i = 0; i < correspondences.size() / 2; i++) {
                int color = ColorUtil.randomColor().getRGB();
                vcd.add(color);
                vcd.add(color);
            }
            VzLines lines = new VzLines(new VisVertexData(correspondences),
                    VzLines.LINES,
                    new VzLines.Style(vcd, 1));
            return lines;
        }

        protected void plotFeatures(ArrayList<ImageFeature> featuresC, BufferedImage rgbC, ArrayList<ImageFeature> featuresL, BufferedImage rgbL) {
            // Plot features
            for (int i = 0; i < featuresC.size(); i++) {
                int[] xy = featuresC.get(i).xy();
                for (int j = -2; j < 3; j++) {
                    for (int k = -2; k < 3; k++) {
                        rgbC.setRGB(xy[0] + j, xy[1] + k, Color.RED.getRGB());
                    }
                }
            }
            for (int i = 0; i < featuresL.size(); i++) {
                int[] xy = featuresL.get(i).xy();
                for (int j = -2; j < 3; j++) {
                    for (int k = -3; k < 3; k++) {
                        rgbL.setRGB(xy[0] + j, xy[1] + k, Color.BLUE.getRGB());
                    }
                }
            }
        }
    }

    static public void main(String[] args) {
        GetOpt opts = new GetOpt();
        opts.addBoolean((char) 0, "point-cloud", false, "Render colored point cloud");
        opts.addBoolean((char) 0, "alignment", false, "Show alignment between features");
        opts.addBoolean('h', "help", false, "Show this help screen");

        if (!opts.parse(args)) {
            System.err.println("ERR: Option failure = " + opts.getReason());
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
        }

        KinectDemo kd = new KinectDemo(opts);
    }
}
