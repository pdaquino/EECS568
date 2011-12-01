package rgbdslam;

/**
 * Thrown when something goes wrong in OpenCV native land.
 * @author pdaquino
 */
public class OpenCVException extends RuntimeException {

    public OpenCVException(String message) {
        super(message);
    }

    public OpenCVException() {
    }
    
    
}
