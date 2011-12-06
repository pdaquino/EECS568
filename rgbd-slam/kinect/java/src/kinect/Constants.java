/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package kinect;

/**
 *
 * @author jrpeterson
 */

// calibration constants for the kinect
public class Constants {
    
    // image dimensions for both rgb and depth images
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;
    
    // John's Calibration Data
    /*
    // RGB Intrinsic Camera Parameters
    static final double Frgbx = 521.67090; // focal lengths
    static final double Frgby = 521.23461;
    static final double Crgbx = 312.82654; // optial axis
    static final double Crgby = 258.60812;
    // assume 0 skew
    // distortion parameters 1 2 and 5 are radial terms, 3 and 4 are tangential
    static final double[] Krgb = {0.18993, -0.52470, 0.00083, 0.00480, 0};
    
    // IR Intrinsic Camera Parameters
    static final double Firx = 583.56911; // focal lengths
    static final double Firy = 582.28721;
    static final double Cirx = 317.73984; // optical axis
    static final double Ciry = 248.91467;
    // assume 0 skew
    // distortion parameters 1 2 and 5 are radial terms, 3 and 4 are tangential
    static final double[] Kir = {-0.09234, 0.31571, 0.00037, -0.00425, 0};
     */
    
    // John's using many more images
    // RGB Intrinsic Camera Parameters
    /*
    static final double Frgbx = 523.06864; // focal lengths
    static final double Frgby = 522.62898;
    static final double Crgbx = 309.46501; // optial axis
    static final double Crgby = 256.30813;
    // assume 0 skew
    // distortion parameters 1 2 and 5 are radial terms, 3 and 4 are tangential
    static final double[] Krgb = {0.19117,   -0.44270,   -0.00323,   0.00064,  0.00000};
    
    // IR Intrinsic Camera Parameters
    static final double Firx = 583.17822; // focal lengths
    static final double Firy = 581.88746; 
    static final double Cirx = 319.98857; // optical axis
    static final double Ciry = 243.38230;
    // assume 0 skew
    // distortion parameters 1 2 and 5 are radial terms, 3 and 4 are tangential
    static final double[] Kir = {-0.07415, 0.17787, 0.00168, -0.00245, 0.00000};
     */
    
    // Camera calibration numbers courtesy of Nicolas Burrus
    // parameters for rgb color camera
    static double Frgbx = 5.2921508098293293e2; // focal length
    static double Frgby = 5.2556393630057437e2;
    static double Crgbx = 3.2894272028759258e2; // camera center in pixels
    static double Crgby = 2.6748068171871557e2;
    static double[] Krgb = {2.6451622333009589e-1,  -8.3990749424620825e-1,
	-1.9922302173693159e-3, 1.4371995932897616e-3, 9.1192465078713847e-1};

    // parameters for IR depth camera
    static double Firx = 5.9421434211923247e2; // focal length
    static double Firy = 5.9104053696870778e2;
    static double Cirx = 3.3930780975300314e2; // camera center in pixels
    static double Ciry = 2.4273913761751615e2;
    static double[] Kir = {-2.6386489753128833e-1, 9.9966832163729757e-1,-7.6275862143610667e-4,
	5.0350940090814270e-3, -1.3053628089976321};
    
    static double[][] Rirtorgb = new double[][] {{9.9984628826577793e-1, 1.2635359098409581e-3, -1.7487233004436643e-2, 0},
                                        {-1.4779096108364480e-3, 9.9992385683542895e-1, -1.2251380107679535e-2, 0},
                                        {1.7470421412464927e-2, 1.2275341476520762e-2, 9.9977202419716948e-1, 0},
                                        {0,0,0,1}};
    
    static double[][] Tirtorgb = new double[][] {{1, 0, 0,1.9985242312092553e-2},
        {0, 1, 0, -7.4423738761617583e-4}, {0, 0, 1, -1.0916736334336222e-2}, {0,0,0,1}};
    /*
    static double[][] trans = LinAlg.translate(new double[] {1.9985242312092553e-2,                                            -7.4423738761617583e-4,                                              -1.0916736334336222e-2});
     */
    
         // Lauren's Code
    /*
    static final int WIDTH = Kinect.WIDTH;
    static final int HEIGHT = Kinect.HEIGHT;

    static double dfx = 5.8e+02;
    static double dfy = 5.8e+02;
    //  static double dcx = 3.1553578317293898e+02; \\Lauren's
    //  static double dcy = 2.4608755771403534e+02; \\Lauren's
    static double dcx = 3.2353578317293898e+02;
    static double dcy = 2.608755771403534e+02;
    double rfx = 5.25e+02;
    double rfy = 5.25e+02;
    double rcx = 3.1924870232372928e+02;
    double rcy = 2.6345521395833958e+02;

    static double[] t = new double[]{-1.5e-02, 2.5073334719943473e-03,-1.2922411623995907e-02};
    */
}
