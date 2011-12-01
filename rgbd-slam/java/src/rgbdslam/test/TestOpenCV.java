package rgbdslam.test;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.io.File;
import java.io.IOException;
import java.util.List;
import javax.imageio.ImageIO;
import rgbdslam.*;

/**
 *
 * @author pdaquino
 */
public class TestOpenCV {
    // pass it the name of an image and it will compute the features in it
    public static void main(String[] args) throws IOException {
        BufferedImage im = ImageIO.read(new File(args[0]));
        BufferedImage rgbIm = new BufferedImage(im.getWidth(),
                    im.getHeight(), BufferedImage.TYPE_INT_RGB);
        rgbIm.getGraphics().drawImage(im, 0, 0, null);
        int[] buf = ((DataBufferInt)(rgbIm.getRaster().getDataBuffer())).getData();
        OpenCV wrapper = new OpenCV();
        List<ImageFeature> features = wrapper.extractFeatures(buf, im.getWidth());
        System.out.println(features.size() + " features found.");
    }
}
