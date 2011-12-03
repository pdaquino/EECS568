package rgbdslam;

/**
 * A feature from an image, such as a corner.
 * @author pdaquino
 */
public class ImageFeature {
    private int[] xy; // pixel coordinates
    private double[] descriptor; // sift descriptor;
    
    public ImageFeature(int x, int y, double[] descriptor) {
        this.xy = new int[] { x, y };
        this.descriptor = descriptor;
    }

    public double[] getDescriptor() {
        return descriptor;
    }

    public int x() {
        return xy[0];
    }

    public int y() {
        return xy[1];
    }
    
    public int[] xy() {
        return xy;
    }
    
    
    
}
