package rgbdslam.test;

/**
 *
 * @author pdaquino
 */
public class DebugPrint {
    static boolean debug = false;
    public static void println(String s) {
        if(debug) {
            System.out.println(s);
        }
    }
}
