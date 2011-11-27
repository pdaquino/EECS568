package kinect;

import java.util.*;

import april.jmat.*;
import april.vis.*;

public class ColorPointCloud
{
    public ArrayList<double[]> points = new ArrayList<double[]>();
    public ArrayList<Integer> colors = new ArrayList<Integer>();
    public VisColorData vcd = new VisColorData();

    static final int WIDTH = Kinect.WIDTH;
    static final int HEIGHT = Kinect.HEIGHT;

    static double dfx = 5.8e+02;
    static double dfy = 5.8e+02;
    static double dcx = 3.1553578317293898e+02;    
    static double dcy = 2.4608755771403534e+02;

    static double[] t = new double[]{-1.5e-02, 2.5073334719943473e-03,-1.2922411623995907e-02};

    public ColorPointCloud(Kinect.Frame frame)
    {
        // XXX Undistort the image!

        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                // Calculate point place in world
                double m = frame.depthToMeters(frame.depth[y*WIDTH + x]);
                if (m < 0)
                    continue;
                double px = ((x - dcx) * m/dfx)  + t[0];
                double py = ((y - dcy) * m/dfy) + t[1];
                double pz = m + t[2];
                points.add(new double[] {pz, -px, -py});     

                // Calculate color of point
                int modx = ((int)(px * rfx/pz + rcx));
                int mody = ((int)(py * rfy/pz + rcy));

                int argb;
                if (modx < 0 || modx >= WIDTH || mody < 0 || mody >= HEIGHT){
                    argb = 0xff000000;
                }
                else{
                    argb = frame.argb[mody*WIDTH + modx];
                }

                colors.add(argb);
                int abgr = (argb & 0xff000000) | ((argb & 0xff) << 16) | (argb & 0xff00) | ((argb >> 16) & 0xff);
                vcd.add(abgr);
            }
        }
    }

    public int numPoints()
    {
        return points.size();
    }
}