package kinect;

import java.util.*;
import java.lang.Math; // for undistortion

import april.jmat.*;
import april.vis.*;

public class ColorPointCloud
{
    public ArrayList<double[]> points = new ArrayList<double[]>();
    public ArrayList<Integer> colors = new ArrayList<Integer>();
    public VisColorData vcd = new VisColorData();
    
    // RGB Intrinsic Camera Parameters
    static final double Frgbx = 521.67090; // focal lengths
    static final double Frgby = 521.23461;
    static final double Crgbx = 300; // optial axis 312.82654
    static final double Crgby = 272; //258.60812
    

    // IR Intrinsic Camera Parameters
    static final double Firx = 583.56911; // focal lengths
    static final double Firy = 582.28721; 
    static final double Cirx = 317.73984; // optical axis
    static final double Ciry = 248.91467;
    
    // Camera calibration numbers courtesy of Nicolas Burrus
    // parameters for rgb color camera
    static double fx_rgb = 5.2921508098293293e2; // focal length
    static double fy_rgb = 5.2556393630057437e2; 
    static double cx_rgb = 3.2894272028759258e2; // camera center in pixels
    static double cy_rgb = 2.6748068171871557e2;
    // the following may be terms in the Brown's Distortion model
    static double k1_rgb = 2.6451622333009589e-1; // radial distortion coefficient
    static double k2_rgb = -8.3990749424620825e-1;
    static double p1_rgb = -1.9922302173693159e-3; // tangential distortion coefficient
    static double p2_rgb = 1.4371995932897616e-3;
    static double k3_rgb = 9.1192465078713847e-1;

    // parameters for IR depth camera
    static double fx_d = 5.9421434211923247e2; // focal length
    static double fy_d = 5.9104053696870778e2;
    static double cx_d = 3.3930780975300314e2; // camera center in pixels
    static double cy_d = 2.4273913761751615e2;
    // the following may be terms in the Brown's Distortion model
    static double k1_d = -2.6386489753128833e-1; // radial distortion coefficient
    static double k2_d = 9.9966832163729757e-1;
    static double p1_d = -7.6275862143610667e-4; // tangential distortion coefficient
    static double p2_d = 5.0350940090814270e-3;
    static double k3_d = -1.3053628089976321;

    /*
    double[][] transform = new double[][] {
[ 9.9984628826577793e-01, 1.2635359098409581e-03,
    -1.7487233004436643e-02, -1.4779096108364480e-03,
    9.9992385683542895e-01, -1.2251380107679535e-02,
    1.7470421412464927e-02, 1.2275341476520762e-02,
    9.9977202419716948e-01 ]
    */

    // rotation transformation between IR depth camera and RGB color camera
    static double[][] rotate = new double[][] {{9.9984628826577793e-1, 1.2635359098409581e-3, -1.7487233004436643e-2, 0},
                                        {-1.4779096108364480e-3, 9.9992385683542895e-1, -1.2251380107679535e-2, 0},
                                        {1.7470421412464927e-2, 1.2275341476520762e-2, 9.9977202419716948e-1, 0},
                                        {0,0,0,1}};
    static double[][] trans = LinAlg.translate(new double[] {1.9985242312092553e-2,
                                                      -7.4423738761617583e-4,
                                                      -1.0916736334336222e-2});

    public ColorPointCloud(Kinect.Frame frame)
    {
        // XXX Undistort the image!
       
        for (int y = 0; y < frame.depthHeight; y++) {
            for (int x = 0; x < frame.depthWidth; x++) {
                // Calculate point place in world
                double m = frame.depthToMeters(frame.depth[y*frame.depthWidth + x]);
                if (m < 0)
                    continue;
		
                
		// undistort depth information for projection into 3D
                // formula courtesy of http://en.wikipedia.org/wiki/Distortion_(optics)
		//double r_d = Math.sqrt((x - cx_d)*(x - cx_d) + (y - cy_d)*(y - cy_d));
                double xu = x;
		//double xu = x + (x - cx_d)*(k1_d*r_d*r_d + k2_d*Math.pow(r_d,4) + k3_d*Math.pow(r_d,6)) +
                //     (p1_d*(r_d*r_d + 2*(x - cx_d)*(x - cx_d)) + 2*p2_d*(x - cx_d)*(y - cy_d));
                double yu = y;
                //double yu = y + (y - cy_d)*(k1_d*r_d*r_d + k2_d*Math.pow(r_d,4) + k3_d*Math.pow(r_d,6)) +
                //     (p2_d*(r_d*r_d + 2*(y - cy_d)*(x - cx_d)) + 2*p1_d*(x - cx_d)*(y - cy_d));
                
                // points in 3D
                double px = (xu - Cirx) * m/Firx;
                double py = (y - Ciry) * m/Firy;
                double pz = m;
		
                
                /*double px = (x - cx_d) * m/fx_d;
                double py = (y - cy_d) * m/fy_d;
                double pz = m; */
                

                // Calculate color of point
                int cx = 0, cy = 0;
                // rotation transformation to transform from IR frame to RGB frame
                double[] xyz = new double[] {px, py, pz};
                double[] cxyz = LinAlg.transform(rotate, xyz);
                cxyz = LinAlg.transform(trans, cxyz);

                // project 3D point into rgb image frame
                cx = (int) ((cxyz[0] * Frgbx / cxyz[2]) + Crgbx);
                cy = (int) ((cxyz[1] * Frgby / cxyz[2]) + Crgby);
  
                /*
                // now need to distort the points to see where they end up in the RGB image
                // or could undistort all of the RGB image and then do the pixel correspondance <= this is better
                // the below implementation attempts to reverse the distortion equation using the actual pixel location
                // as an approximation of the distorted location but this isn't as accurate
                r_rgb = Math.sqrt((cx - cx_rgb)*(cx - cx_rgb) + (cy - cy_rgb)*(cy - cy_rgb));
		cxd = cx - (cx - cx_rgb)*(k1_rgb*r_rgb*r_rgb + k2_rgb*Math.pow(r_rgb,4) + k3_rgb*Math.pow(r_rgb,6)) -
                     (p1_rgb*(r_rgb*r_rgb + 2*(cx - cx_rgb)*(cx - cx_rgb)) + 2*p2_rgb*(cx - cx_rgb)*(cy - cy_rgb));
	        cyd = cy - (cy - cy_rgb)*(k1_rgb*r_rgb*r_rgb + k2_rgb*Math.pow(r_rgb,4) + k3_rgb*Math.pow(r_rgb,6)) -
                     (p1_rgb*(r_rgb*r_rgb + 2*(cy - cy_rgb)*(cy - cy_rgb)) + 2*p2_rgb*(cx - cx_rgb)*(cy - cy_rgb));
                cx = cxd;
                cy = cyd; 
                // need a way to handle points that end up outside of the color image, could return just a gray value
                // and also want a way to handle points from color image that don't get depth information
                */

                assert (!(cx < 0 || cx > frame.rgbWidth));
                assert (!(cy < 0 || cy > frame.rgbHeight));

                points.add(new double[] {px, py, pz});
                int argb = frame.argb[cy*frame.rgbWidth + cx]; // get the rgb data for the calculated pixel location
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

