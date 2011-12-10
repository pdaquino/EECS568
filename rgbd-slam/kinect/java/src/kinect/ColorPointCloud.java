package kinect;

import java.util.*;

import april.jmat.*;
import april.vis.*;
import kinect.Kinect.Frame;

public class ColorPointCloud {

    public ArrayList<double[]> points;
    public ArrayList<Integer> colors;
    public VisColorData vcd = new VisColorData();
    // Array stores mapping between pixels in rgb image and points in the point cloud
    int[] pointmap;

    private void buildArrays() {
        points = new ArrayList<double[]>(307200);
        colors = new ArrayList<Integer>(307200);
        pointmap = new int[Constants.HEIGHT * Constants.WIDTH];
    }

    public ColorPointCloud(Kinect.Frame frame) {
        buildArrays();
        for (int y = 0; y < Constants.HEIGHT; y++) {
            for (int x = 0; x < Constants.WIDTH; x++) {
                processPoint(frame, x, y);
            }
        }
    }

    // alternate constructor for making a decimated point cloud
    // 1 is full resolution, 10 would be every 10 pixels
    // XXX fix for aliasing!
    public ColorPointCloud(Kinect.Frame frame, int Dfactor) {
        buildArrays();
        for (int y = 0; y < Constants.HEIGHT; y = y + Dfactor) {
            for (int x = 0; x < Constants.WIDTH; x = x + Dfactor) {
                processPoint(frame, x, y);
            }
        }
    }

    protected final void processPoint(Frame frame, int x, int y) {
        
        // Calculate point place in world
        double m = frame.depthToMeters(frame.depth[y * Constants.WIDTH + x]);
        if (m < 0 || m > 3.8) {
            return; // give up if got a depth outside of our accurate range
        }

        // points in 3D
        double px = (x - Constants.Cirx) * m / Constants.Firx;
        double py = (y - Constants.Ciry) * m / Constants.Firy;
        double pz = m;

        // Calculate color of point
        int cx = 0, cy = 0;

        // rotation transformation to transform from IR frame to RGB frame
        double[] xyz = new double[]{px, py, pz};
        double[] cxyz = LinAlg.transform(Constants.Transirtorgb, xyz);

        // project 3D point into rgb image frame
        cx = (int) ((cxyz[0] * Constants.Frgbx / cxyz[2]) + Constants.Crgbx);
        cy = (int) ((cxyz[1] * Constants.Frgby / cxyz[2]) + Constants.Crgby);
        assert (!(cx < 0 || cx > Constants.WIDTH));
        assert (!(cy < 0 || cy > Constants.HEIGHT));

        points.add(new double[]{px, py, pz});

        pointmap[cy * Constants.WIDTH + cx] = points.size() - 1;

        int argb = frame.argb[cy * Constants.WIDTH + cx]; // get the rgb data for the calculated pixel location

        if (pz < 1000) {
            colors.add(argb);
            int abgr = (argb & 0xff000000) | ((argb & 0xff) << 16) | (argb & 0xff00) | ((argb >> 16) & 0xff);
            vcd.add(abgr);
        }
    }

    // projects image coordinates in rgb image into 3D space
    public double[] Project(double[] Xrgb) {
        assert (Xrgb.length == 2);

        int index = pointmap[(int) (Xrgb[1] * Constants.WIDTH + Xrgb[0])];

        if (index == 0) { // if this part of the point map was never initialized
            return new double[]{-1, -1, -1};
        }

        double[] P = this.points.get(index);

        assert (P.length == 3) : "Warning Expected found point to be in 3D";
        return P;
    }

    public ArrayList<double[]> Project(List<double[]> XRGB) {
        ArrayList<double[]> P = new ArrayList<double[]>();

        for (double[] Xrgb : XRGB) {
            double[] p = Project(Xrgb);
            P.add(p);
        }
        return P;
    }

    public int numPoints() {
        return points.size();
    }
}
