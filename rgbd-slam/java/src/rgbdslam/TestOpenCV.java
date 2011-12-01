package rgbdslam;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;

/**
 *
 * @author pdaquino
 */
public class TestOpenCV {
    public static void main(String[] args) throws IOException {
        BufferedImage im = ImageIO.read(new File("bighouse.jpg"));
        BufferedImage rgbIm = new BufferedImage(im.getWidth(),
                    im.getHeight(), BufferedImage.TYPE_INT_RGB);
        rgbIm.getGraphics().drawImage(im, 0, 0, null);
        int[] buf = ((DataBufferInt)(rgbIm.getRaster().getDataBuffer())).getData();
        OpenCV wrapper = new OpenCV();
        wrapper.extractFeatures(buf, im.getWidth(), 20);
    }
}
