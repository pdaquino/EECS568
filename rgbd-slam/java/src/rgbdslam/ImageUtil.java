package rgbdslam;

/**
  
 * @author pdaquino
 */
public class ImageUtil {
    public static double[] flipXY(double[] xy, double imageHeight) {
        return new double[] { xy[0], imageHeight-xy[1] };
    }
}
