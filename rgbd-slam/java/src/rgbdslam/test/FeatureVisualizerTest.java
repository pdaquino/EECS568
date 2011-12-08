package rgbdslam.test;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.io.*;
import javax.imageio.ImageIO;
import rgbdslam.*;
import rgbdslam.DescriptorMatcher.Match;
import rgbdslam.vis.FeatureVisualizer;

/**
 *
 * @author pdaquino
 */
public class FeatureVisualizerTest {
    
    public static void main(String[] args) throws IOException {
        FeatureVisualizer visualizer = new FeatureVisualizer();
        BufferedImage im1 = getRGBImage(args[0]);
        BufferedImage im2 = getRGBImage(args[1]);
        
        java.util.List<ImageFeature> features1 = OpenCV.extractFeatures(getImageAsIntArray(im1), im1.getWidth());
        java.util.List<ImageFeature> features2 = OpenCV.extractFeatures(getImageAsIntArray(im2), im2.getWidth());
        java.util.List<Match> matches = new DescriptorMatcher(features1, features2).match();
        
        visualizer.updateFrames(im1, im2, matches);

    }
    
    protected static int[] getImageAsIntArray(BufferedImage rgbIm) {
        return ((DataBufferInt) (rgbIm.getRaster().getDataBuffer())).getData();
    }
    
    protected static BufferedImage getRGBImage(String filename) throws IOException {
        BufferedImage im = ImageIO.read(new File(filename));
        BufferedImage rgbIm = new BufferedImage(im.getWidth(),
                im.getHeight(), BufferedImage.TYPE_INT_RGB);
        rgbIm.getGraphics().drawImage(im, 0, 0, null);
        return rgbIm;
    }
}
