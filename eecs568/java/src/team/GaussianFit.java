package team;

import java.util.*;
import java.awt.*;
import javax.swing.*;

import april.jmat.*;
import april.vis.*;
import april.util.*;

/** Sample 500 obsercations, transform into euclidean space. Plot
 *  positions in red. Fit a MG to the points and plot a 3-sigma
 *  contour in red.
 *
 *  Compute the covariance matrix by using covariance projection.
 *  Plot the 3-sigma contour in yellow.
 *
 *  Repeat the above for varying noise levels.
 */
class GaussianFit
{
    static public void main(String[] args)
    {
        JFrame jf = new JFrame("Fitting Gaussians PS1 - 3B");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800,600);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);

        // Force a grid to screen
        VisGrid vg = new VisGrid();
        VisWorld.Buffer vbGrid = vw.getBuffer("grid");
        vbGrid.setDrawOrder(-1000);
        vbGrid.addBack(new VisDepthTest(false, vg));
        vbGrid.swap();

        final VisWorld.Buffer vbPoints = vw.getBuffer("points");
        final VisWorld.Buffer vbContours = vw.getBuffer("contours");

        jf.add(vc, BorderLayout.CENTER);

        ParameterGUI pg = new ParameterGUI();
        pg.addButtons("samples", "Collect Samples");
        pg.addDouble("sig2r", "r variance", 50);
        pg.addDouble("sig2theta", "theta variance", 0.15);
        pg.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name)
            {
                if (name.equals("samples")) {
                    double r, t;
                    r = pg.gd("sig2r");
                    t = pg.gd("sig2theta");
                    samplePoints(vbPoints, vbContours, r, t);
                }
            }
        });

        jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }

    static void samplePoints(VisWorld.Buffer vbp,
                             VisWorld.Buffer vbc,
                             double sig2r,
                             double sig2theta)
    {
        // Sample many points with noise levels specified by sig2r, sig2theta
        // XXX Arbitrary sampling point, (100, Pi/4)
        double r_hat = 100.0;
        double theta_hat = Math.PI/8.0;
        double[] mu = new double[] {r_hat, theta_hat};
        double[][] Sigma = new double[][] { {sig2r, 0},
                                            {0, sig2theta} };
        MultiGaussian mg = new MultiGaussian(Sigma, mu);

        final int NUM_SAMPLES = 500;
        ArrayList<double[]> samples = new ArrayList<double[]>();
        Random random = new Random();
        for (int i = 0; i < NUM_SAMPLES; i++) {
            samples.add(mg.sample(random));
        }

        // Project these samples into Euclidean (x,y) space
        ArrayList<double[]> xysamples = new ArrayList<double[]>();
        for (double[] sample: samples) {
            double r = sample[0];
            double t = sample[1];
            double[] xysample = new double[] { r*Math.cos(t),
                                               r*Math.sin(t) };
            xysamples.add(xysample);
        }

        // Plot points in red
        VisVertexData points = new VisVertexData(xysamples);
        VisConstantColor vccRed = new VisConstantColor(Color.red);
        VisConstantColor vccYellow = new VisConstantColor(Color.yellow);

        vbp.addBack(new VisLighting(false, new VisPoints(points,
                                                         vccRed,
                                                         2.0)));
        vbp.swap();

        // Fit a contour to the samples
        MultiGaussian mgFit = new MultiGaussian(xysamples);
        ArrayList<double[]> contour = mgFit.getContour(9.0);    // 3 sigma?
        VisVertexData contourPoints = new VisVertexData(contour);
        vbc.addBack(new VisLighting(false, new VisLines(contourPoints,
                                                        vccRed,
                                                        2.0,
                                                        VisLines.TYPE.LINE_LOOP)));

        // Use covariance projection to compute the Euclidean version
        // of the covariance matrix. Plot the 3-sigma contour for
        // this as well
        double[] mu_xy = new double[] {r_hat*Math.cos(theta_hat),
                                       r_hat*Math.sin(theta_hat)};
        double ct = Math.cos(theta_hat);
        double st = Math.sin(theta_hat);
        double xy00 = sig2r*ct*ct + sig2theta*r_hat*r_hat*st*st;
        double xy10 = sig2r*ct*st - sig2theta*r_hat*r_hat*ct*st;
        double xy01 = sig2r*ct*st - sig2theta*r_hat*r_hat*ct*st;
        double xy11 = sig2r*st*st + sig2theta*r_hat*r_hat*ct*ct;
        double[][] Sigma_xy = new double[][] { {xy00, xy10},
                                               {xy01, xy11} };

        MultiGaussian mgProject = new MultiGaussian(Sigma_xy, mu_xy);
        ArrayList<double[]> contourProject = mgProject.getContour(9.0);
        VisVertexData projectPoints = new VisVertexData(contourProject);
        vbc.addBack(new VisLighting(false, new VisLines(projectPoints,
                                                        vccYellow,
                                                        2.0,
                                                        VisLines.TYPE.LINE_LOOP)));

        vbc.swap();

    }
}
