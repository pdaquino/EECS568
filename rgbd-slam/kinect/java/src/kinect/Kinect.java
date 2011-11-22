package kinect;

import java.util.*;
import java.awt.image.*;

public class Kinect
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

        public BufferedImage makeRGB()
        {
            assert (argb.length == rgbHeight*rgbWidth);
            BufferedImage im = new BufferedImage(rgbWidth, rgbHeight, BufferedImage.TYPE_INT_RGB);
            int[] buf = ((DataBufferInt)(im.getRaster().getDataBuffer())).getData();
            for (int i = 0; i < buf.length; i++) {
                buf[i] = argb[i];
            }

            return im;
        }

        public BufferedImage makeDepth()
        {
            assert (depth.length == depthHeight*depthWidth);
            BufferedImage im = new BufferedImage(depthWidth, depthHeight, BufferedImage.TYPE_INT_RGB);
            int[] buf = ((DataBufferInt)(im.getRaster().getDataBuffer())).getData();
            double[] cutoffs = new double[] {1.0, 1.75, 2.5, 3.25, 4.0, 5.0};
            for (int i = 0; i < buf.length; i++) {
                // XXX Improved color mapping. Optimal range is ~0.8m - 3.5m
                // white -> close
                // red
                // orange
                // yellow
                // green
                // blue
                // magenta
                // black -> bad values
                double m = depthToMeters(depth[i]);
                if (m < 0) {
                    buf[i] = 0;
                    continue;
                }
                int r,g,b;
                if (m < cutoffs[0]) {
                    r = 0xff;
                    g = 0xff - (int) (0xff * m/cutoffs[0]);
                    b = 0xff - (int) (0xff * m/cutoffs[0]);
                } else if (m < cutoffs[1]) {
                    r = 0xff;
                    g = 0xff - (int) (0xff * ((cutoffs[1] - m)/(cutoffs[1]-cutoffs[0])));
                    b = 0;
                } else if (m < cutoffs[2]) {
                    r = (int) (0xff * ((cutoffs[2] - m)/(cutoffs[2]-cutoffs[1])));
                    g = 0xff;
                    b = 0;
                } else if (m < cutoffs[3]) {
                    r = 0;
                    g = (int) (0xff * ((cutoffs[3] - m)/(cutoffs[3]-cutoffs[2])));
                    b = 0xff - (int) (0xff * ((cutoffs[3] - m)/(cutoffs[3]-cutoffs[2])));
                } else if (m < cutoffs[4]) {
                    r = 0xff - (int) (0xff * ((cutoffs[4] - m)/(cutoffs[4]-cutoffs[3])));
                    g = 0;
                    b = 0xff;
                } else if (m < cutoffs[5]) {
                    r = (int) (0xff * ((cutoffs[5] - m)/(cutoffs[5]-cutoffs[4])));
                    g = 0;
                    b = (int) (0xff * ((cutoffs[5] - m)/(cutoffs[5]-cutoffs[4])));
                } else {
                    r = 0;
                    g = 0;
                    b = 0;
                }

                buf[i] = 0xff000000 | (r << 16) | (g << 8) | b;
            }

            return im;
        }
    }
}
