package rgbdslam;

/**
 * A feature from an image, such as a corner.
 * @author pdaquino
 */
public class ImageFeature {
    private int[] xy; // pixel coordinates
    private double[] descriptor; // sift descriptor;
    private double[] xyz; // world coordinates

    /**
     * Constructs an image feature.
     * @param x the x coordinate of the feature, in the image coordinate frame (pixels)
     * @param y the y coordinate of the feature, in the image coordinate frame (pixels)
     * @param descriptor the feature descriptor, such as a SIFT descriptor
     */
    public ImageFeature(int x, int y, double[] descriptor) {
        this.xy = new int[] { x, y };
        this.descriptor = descriptor;
    }

    /**
     * Returns the feature descriptor (e.g. SIFT descriptor).
     * @return
     */
    public double[] getDescriptor() {
        return descriptor;
    }

    /**
     * Returns the pixel coordinates of the feature in the image.
     * @return
     */
    public int[] xy() {
        return xy;
    }

    public double[] xyAsDouble() {
        return new double[] { xy[0], xy[1] };
    }

    /**
     * Returns the position of the feature in world coordinates, with depth
     * information.
     * @return
     */
    public double[] xyz() {
        return xyz;
    }

    /**
     * Sets the world coordinates of the feature.
     * @param xyz
     */
    public void setXyz(double[] xyz) {
        this.xyz = xyz;
    }



}
