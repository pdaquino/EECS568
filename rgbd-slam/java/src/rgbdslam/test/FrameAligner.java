package rgbdslam.test;

import april.jmat.LinAlg;
import april.util.ParameterGUI;
import april.util.ParameterListener;
import april.vis.*;
import april.vis.VisWorld.Buffer;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.util.List;
import javax.swing.JFrame;
import kinect.*;
import rgbdslam.AlignFrames;
import rgbdslam.ImageFeature;
import rgbdslam.ImageUtil;
import rgbdslam.OpenCV;

/**
 *
 * @author pdaquino
 */
public class FrameAligner {

    private Kinect.Frame frame1, frame2;

    public FrameAligner(Kinect.Frame frame1, Kinect.Frame frame2) {
        initFrame();
        this.frame1 = frame1;
        this.frame2 = frame2;
        showFrames();
    }
    private VisWorld vw = new VisWorld();
    private VisLayer vl = new VisLayer(vw);
    private VisCanvas vc = new VisCanvas(vl);
    private JFrame jf = new JFrame("Feature Visualizer");
    private ParameterGUI pg = new ParameterGUI();
    public static final int IMAGE_SEPARATION = 10;

    protected final void initFrame() {
        int windowHeight = Constants.HEIGHT * 2 + IMAGE_SEPARATION;
        int windowWidth = Constants.WIDTH * 2 + IMAGE_SEPARATION;
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(windowWidth, windowHeight);
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        pg.addBoolean("superimpose-rgb", "Superimpose RGB images", false);
        pg.addListener(new ParameterListener() {

            public void parameterChanged(ParameterGUI pgui, String name) {
                if (name.equals("superimpose-rgb")) {
                    superimposeRGB();
                }
            }
        });

        jf.setVisible(true);
        vl.cameraManager.fit2D(new double[]{0, 0}, new double[]{windowWidth, windowHeight}, true);
    }

    private void superimposeRGB() {
    }

    private void showFrames() {
        BufferedImage im1 = frame1.makeRGB();
        BufferedImage im2 = frame2.makeRGB();
        BufferedImage depth1 = frame1.makeDepth();
        BufferedImage depth2 = frame2.makeDepth();
        ColorPointCloud pts1 = new ColorPointCloud(frame1);
        ColorPointCloud pts2 = new ColorPointCloud(frame2);
        List<ImageFeature> feats1 = AlignFrames.extractAndProjectFeatures(frame1, pts1);
        List<ImageFeature> feats2 = AlignFrames.extractAndProjectFeatures(frame2, pts2);

        {
            VisWorld.Buffer vbIm = vw.getBuffer("rgb-images");
            vbIm.addBack(new VisChain(LinAlg.translate(0, Constants.HEIGHT + IMAGE_SEPARATION),
                    new VzImage(im1, VzImage.FLIP)));
            vbIm.addBack(new VisChain(LinAlg.translate(Constants.WIDTH + IMAGE_SEPARATION, Constants.HEIGHT + IMAGE_SEPARATION),
                    new VzImage(im2, VzImage.FLIP)));
            vbIm.swap();
        }

        {
            VisWorld.Buffer vbIm = vw.getBuffer("depth-images");
            vbIm.addBack(new VisChain(LinAlg.translate(0, 0),
                    new VzImage(depth1, VzImage.FLIP)));
            vbIm.addBack(new VisChain(LinAlg.translate(Constants.WIDTH + IMAGE_SEPARATION, 0),
                    new VzImage(depth2, VzImage.FLIP)));
            vbIm.swap();
        }

        {
            VisWorld.Buffer vbIm = vw.getBuffer("features");
            drawFeatures(vbIm, feats1, new double[] { 0, Constants.HEIGHT + IMAGE_SEPARATION, 0}, Color.red);
            drawFeatures(vbIm, feats2, new double[] { Constants.WIDTH + IMAGE_SEPARATION, Constants.HEIGHT + IMAGE_SEPARATION, 0},
                    Color.blue);
            vbIm.swap();
        }
        
        {
            VisWorld.Buffer vb = vw.getBuffer("pts-clouds");
            vb.addBack(new VzPoints(new VisVertexData(pts1.points), new VzPoints.Style(pts1.vcd, 2)));
            vb.swap();
        }
        
        AlignFrames af = new AlignFrames(frame1, frame2);
        LinAlg.print(af.align(LinAlg.identity(4)).rbt);
    }

    public static void main(String[] args) throws Exception {
        if (args.length < 2) {
            System.err.println("Usage: FrameAligner frame1.kframe frame2.kframe");
            return;
        }
        Kinect.Frame frame1 = loadFrame(args[0]);
        Kinect.Frame frame2 = loadFrame(args[1]);
        FrameAligner aligner = new FrameAligner(frame1, frame2);
    }

    private static Kinect.Frame loadFrame(String filename) throws IOException, ClassNotFoundException {
        FileInputStream is = new FileInputStream(filename);
        ObjectInputStream objis = new ObjectInputStream(is);
        return (Kinect.Frame) objis.readObject();
    }

    private void drawFeatures(Buffer vbIm, List<ImageFeature> feats, double[] translation, Color c) {
        for (ImageFeature feat : feats) {
            double[] xy = ImageUtil.flipXY(feat.xyAsDouble(), Constants.HEIGHT);
            vbIm.addBack(new VisChain(
                    LinAlg.translate(translation),
                    new VzPoints(new VisVertexData(new double[] { xy[0], xy[1], feat.xyz()[2]*10 }),
                    new VzPoints.Style(c, 5))));
        }
    }
}
