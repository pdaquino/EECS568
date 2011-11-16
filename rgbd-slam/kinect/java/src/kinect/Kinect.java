package kinect;

import java.util.*;

class Kinect
{
    static final int RGB_WIDTH = 640;
    static final int RGB_HEIGHT = 480;
    static final int DEPTH_WIDTH = 632;
    static final int DEPTH_HEIGHT = 480;

    // Frame buffers
    int[] rgb_buf = null;
    short[] d_buf = null;

    int rgb_cnt = 0;
    int d_cnt = 0;

    // Initialize the kinect device, returning a negative
    // error code upon failure
    public native int initKinect();
    public native int closeKinect();
    public native void startVideo();
    public native void stopVideo();
    public native void startDepth();
    public native void stopDepth();
    public native int[] getVideoFrame();
    public native short[] getDepthFrame();

    static
    {
        System.loadLibrary("kinect");
    }

    public synchronized int init()
    {
        return initKinect();
    }

    public synchronized int close()
    {
        return closeKinect();
    }

    public synchronized void start()
    {
        startVideo();
        startDepth();
    }

    public synchronized void stop()
    {
        stopVideo();
        stopDepth();
    }

    public synchronized Frame getFrame()
    {
        int[] argb = getVideoFrame();
        short[] depth = getDepthFrame();
        if (argb != null) {
            rgb_buf = argb;
            rgb_cnt++;
        }
        if (depth != null) {
            d_buf = depth;
            d_cnt++;
        }

        if (rgb_buf != null && d_buf != null) {
            Frame f = new Frame(rgb_buf, d_buf);
            rgb_buf = null;
            d_buf = null;

            return f;
        }
        return null;
    }

    public synchronized void printCount()
    {
        System.out.printf("rgb: %d depth: %d\n", rgb_cnt, d_cnt);
    }

    // Practical resolution of depth seems to be:
    // 632 x 480
    static public class Frame
    {
        // Not an ideal location for more constants
        static final public int rgbWidth = 640;
        static final public int rgbHeight = 480;
        static final public int depthWidth = 640;
        static final public int depthHeight = 480;

        public int[] argb;
        public short[] depth;

        static double[] t_gamma = null;
        public Frame(int argb[], short[] depth)
        {
            this.argb = argb;
            this.depth = depth;

            if (t_gamma == null) {
                this.t_gamma = new double[2048];

                // From Stephane Magnenat
                double k1 = 1.1863;
                double k2 = 2842.5;
                double k3 = 0.1236;
                for (int i = 0; i < 2048; i++) {
                    t_gamma[i] = k3*Math.tan(i/k2 + k1);
                }

                // From Willow Garage
                //for (i = 0; i < 2048; i++) {
                //    double frac = i/2048.0;
                //    frac = 6*Math.pow(frac, 3);
                //    t_gamma[i] = v*6*256;
                //}
            }
        }

        public double depthToMeters(short depth)
        {
            // Throw away extreme values
            if ((int) depth == 2048)
                return -1;
            return t_gamma[depth];
        }
    }
}
