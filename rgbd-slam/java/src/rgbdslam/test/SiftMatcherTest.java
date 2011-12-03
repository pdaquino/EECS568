package rgbdslam.test;

import april.util.Tic;
import java.awt.image.*;
import java.io.*;
import java.util.Arrays;
import java.util.List;
import javax.imageio.ImageIO;
import rgbdslam.*;

/**
 *
 * @author pdaquino
 */
public class SiftMatcherTest {
    // pass it the name of an image and it will compute the features in it

    public static void main(String[] args) throws IOException {
        // XXX this is a pretty dumb test, but it's quick and checks if the
        // implementation is obviously wrong. however, it does not help a tiny
        // bit in assessing whether the matching is good enough. to do that
        // we'd need to 1) not use the same image twice 2) display the two
        // images side to side, with lines connecting matching features

        BufferedImage im = ImageIO.read(new File(args[0]));
        BufferedImage rgbIm = new BufferedImage(im.getWidth(),
                im.getHeight(), BufferedImage.TYPE_INT_RGB);
        rgbIm.getGraphics().drawImage(im, 0, 0, null);
        int[] buf = ((DataBufferInt) (rgbIm.getRaster().getDataBuffer())).getData();

        OpenCV wrapper = new OpenCV();
        double timeMatching = 0;
        double timeExtraction = 0;
        
        // make n = 1 if you just want a quick correctness test
        int n = 100;
        for (int i = 0; i < n; i++) {
            Tic tic = new Tic();
            List<ImageFeature> features1 = wrapper.extractFeatures(buf, im.getWidth());
            List<ImageFeature> features2 = wrapper.extractFeatures(buf, im.getWidth());
            timeExtraction += tic.toc();

            System.out.println(features1.size() + " features found.");

            tic = new Tic();
            List<SiftMatcher.Match> matches = new SiftMatcher(features1).match(features2);
            for (SiftMatcher.Match match : matches) {
                if (!Arrays.equals(match.feature1.xy(), match.feature2.xy())) {
                    System.err.println("Wrong match!");
                }
            }
            timeMatching += tic.toc();
        }

        System.out.printf("Extraction: %.5f\nMatching: %.5f\n", timeExtraction/n, timeMatching/n);

    }
}
