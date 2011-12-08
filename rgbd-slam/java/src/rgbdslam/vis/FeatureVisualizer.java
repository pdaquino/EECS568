package rgbdslam.vis;

import april.jmat.LinAlg;
import april.vis.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.io.*;
import javax.swing.*;
import kinect.*;
import rgbdslam.*;
import java.util.*;
import javax.imageio.ImageIO;
import rgbdslam.DescriptorMatcher.Match;

/**
 *
 * @author pdaquino
 */
public class FeatureVisualizer {
    
    public FeatureVisualizer() {
        initFrame();
    }
    
    private VisWorld vw = new VisWorld();
    private VisLayer vl = new VisLayer(vw);
    private VisCanvas vc = new VisCanvas(vl);
    private JFrame jf = new JFrame("Feature Visualizer");
    
    public static final int IMAGE_SEPARATION = 10;

    protected final void initFrame() {
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(Constants.WIDTH*2 + IMAGE_SEPARATION, 500);
        jf.add(vc, BorderLayout.CENTER);

        jf.setVisible(true);
    }
    
    public void updateFrames(BufferedImage im1, BufferedImage im2, java.util.List<Match> matches) {
        
        int imageOffset = im1.getWidth() + IMAGE_SEPARATION;
        {
            VisWorld.Buffer vbIm = vw.getBuffer("image");
            vbIm.addBack(new VzImage(im1, VzImage.FLIP));
            vbIm.addBack(translate(imageOffset, new VzImage(im2, VzImage.FLIP)));

            ArrayList<double[]> correspondences = new ArrayList<double[]>();
            
            double totalDescriptorError = 0.0, total3DError = 0.0;
            for (Match match : matches) {
                double[] xy1 = ImageUtil.flipXY(match.feature1.xyAsDouble(), im1.getHeight());
                vbIm.addBack(new VzPoints(new VisVertexData(xy1),
                        new VzPoints.Style(Color.red, 5)));
                
                double[] xy2 = ImageUtil.flipXY(match.feature2.xyAsDouble(), im2.getHeight());
                xy2[0] += imageOffset;
                vbIm.addBack(new VzPoints(new VisVertexData(xy2),
                        new VzPoints.Style(Color.blue, 5)));
                
                correspondences.add(xy1);
                correspondences.add(xy2);
                
                totalDescriptorError += match.descriptorDistance;
                total3DError += match.xyzDistance;
            }
            
            VisColorData vcd = new VisColorData();
            for (int i = 0; i < correspondences.size()/2; i++) {
                int c = ColorUtil.randomColor(255).getRGB();
                vcd.add(c);
                vcd.add(c);
            }

            VzLines lines = new VzLines(new VisVertexData(correspondences),
                    VzLines.LINES,
                    new VzLines.Style(vcd, 2));
            
            StringBuilder sb = new StringBuilder("<<sansserif-12,dropshadow=true>>");
            sb.append(matches.size()).append(" matches\n");
            sb.append("Average 3D error: ").append(total3DError/matches.size()).append(" m\n");
            sb.append("Average descriptor error: ").append(totalDescriptorError/matches.size()).append("\n");
            
            vbIm.addBack(lines);
            vbIm.addBack(new VzText(VzText.ANCHOR.BOTTOM_LEFT, sb.toString()));

            vbIm.swap();
        }
    }
    
    protected static VisChain translate(double translation, VisObject... os) {
        Object[] objects = new Object[os.length + 1];
        objects[0] = LinAlg.translate(translation, 0);
        System.arraycopy(os, 0, objects, 1, os.length);

        return new VisChain(objects);
    }
}
