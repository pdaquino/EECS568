package rgbdslam;

/**
 * A feature from an image, such as a corner.
 * @author pdaquino
 */
public class ImageFeature {
    private int x, y; // pixel coordinates
    private double[] descriptor; // sift descriptor;
    
    public ImageFeature(int x, int y, double[] descriptor) {
        this.x = x;
        this.y = y;
        this.descriptor = descriptor;
    }

    public double[] getDescriptor() {
        return descriptor;
    }

    public int x() {
        return x;
    }

    public int y() {
        return y;
    }
    
    
    
}
