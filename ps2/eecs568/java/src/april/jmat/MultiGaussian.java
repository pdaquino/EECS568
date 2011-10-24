package april.jmat;

import java.util.*;

import java.awt.*;
import javax.swing.*;

import april.jmat.*;
import april.vis.*;
import april.util.*;

public class MultiGaussian
{
    double P[][];
    double Psqrt[][];
    double u[];
    double PInv[][];

    double pScale;

    /** Create a new MultiGaussian object with covariance P and mean u.
        (This should be trivial)
    **/
    public MultiGaussian(double _P[][], double _u[])
    {
        P = _P;
        u = _u;
        PInv = LinAlg.inverse(P);
        Psqrt = new CholeskyDecomposition(new Matrix(P)).getL().copyArray();

        pScale = 1.0/Math.pow(2*Math.PI, u.length/2.0)/Math.sqrt(LinAlg.det(P));
    }

    /** Create a new MultiGaussian object that estimates the covariance and mean
        from the provided samples. This should implement your algorithm from A. **/
    public MultiGaussian(ArrayList<double[]> samples)
    {
        // Compute mean
        u =  new double[samples.get(0).length];
        for (double pt[] : samples)
            LinAlg.plusEquals(u,pt);
        u = LinAlg.scale(u, 1.0/samples.size());

        // now compute the biased covariance estimate (assume samples are gaussian)
        P = new double[u.length][u.length];
        for (double ptmu[] : samples) {
            double pt[] = LinAlg.subtract(ptmu, u);

            for (int i = 0; i < u.length; i++)
                for (int j = 0; j < u.length; j++)
                    P[i][j] += pt[i]*pt[j];
        }
        P = LinAlg.scale(P, 1.0 / samples.size());


        PInv = LinAlg.inverse(P);
        Psqrt = new CholeskyDecomposition(new Matrix(P)).getL().copyArray();

        pScale = 1.0/Math.pow(2*Math.PI, u.length/2.0)/Math.sqrt(LinAlg.det(P));
    }

    /** Return the covariance associated with this object. (Trivial). **/
    public double[][] getCovariance()
    {
        return P;
    }

    /** Return the mean associated with this object. (Trivial). **/
    public double[] getMean()
    {
        return u;
    }

    /** Draw a random sample from this distribution. This should implement your
        method from part C.
    **/
    public double[] sample(Random r)
    {
        double rand[] = new double[Psqrt.length];
        for (int i = 0; i < rand.length; i++)
            rand[i] = r.nextGaussian();

        return LinAlg.add(LinAlg.matrixAB(Psqrt,rand),u);
    }

    /** Compute the probability of sample x[] wrt to this distribution
     **/
    public double prob(double[] x)
    {
        return pScale*Math.exp(-chi2(x)/2);
    }


    /** Given an observation from the distribution, compute the chi^2 value. This
        is given by (x-u)'inv(M)(x-u)
    **/
    public double chi2(double[] x)
    {
        double xmu[] = LinAlg.subtract(x,u);
        double xmut[][] = new double[][]{xmu};
        return LinAlg.matrixABCt(xmut,PInv,xmut)[0][0];
    }

    /** Compute a set of points that, when plotted as a curve, would trace out an
        iso-probability contour corresponding to the specified chi^2 value. Generate
        points at one-degree spacings using your method from part D.
    **/
    public ArrayList<double[]> getContour(double chi2)
    {
        assert(P.length == 2);

        ArrayList<double[]> points = new ArrayList<double[]>();
        for (int deg = 0; deg <= 360; deg++) {
            double ct = Math.cos(Math.toRadians(deg));
            double st = Math.sin(Math.toRadians(deg));

            double cst[][] = {{ct}, {st}};
            double chi2cst = LinAlg.matrixAtBC(cst,PInv,cst)[0][0];
            double alpha = Math.sqrt(chi2/chi2cst);
            points.add(LinAlg.add(u, LinAlg.scale(new double[]{ct,st}, alpha)));
        }
        return points;
    }

    public static void main(String args[])
    {
        final VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);

        ParameterGUI pg = new ParameterGUI();
        pg.addDoubleSlider("theta","Theta: ", -Math.PI, Math.PI, 0);
        pg.addDoubleSlider("rx","Radius X: ", 0, 10, 3.0);
        pg.addDoubleSlider("ry","Radius Y: ", 0, 10, 1.0);
        pg.addIntSlider("n","Num Samples ", 3, 10000, 100);

        pg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name)
                {
                    double mu[] = {1,3};
                    double P[][] = {{pg.gd("rx")*pg.gd("rx"),0},
                                    {0,pg.gd("ry")*pg.gd("ry")}};

                    // Note this rotation matrix is inverted:
                    double rot[][] = {{Math.cos(pg.gd("theta")),Math.sin(pg.gd("theta"))},
                                      {-Math.sin(pg.gd("theta")), Math.cos(pg.gd("theta"))}};

                    double Prot[][] = LinAlg.matrixAtBC(rot,P,rot);


                    MultiGaussian mg = new MultiGaussian(Prot, mu);

                    Random r = new Random(-24);
                    ArrayList<double[]> samples = new ArrayList<double[]>();
                    for (int i = 0; i < pg.gi("n"); i++)
                        samples.add(mg.sample(r));


                    {
                        VisWorld.Buffer vb = vw.getBuffer("ellipse-orig");

                        vb.addBack(new VisLines(new VisVertexData(mg.getContour(9)),
                                                new VisConstantColor(Color.yellow),
                                                3,
                                                VisLines.TYPE.LINE_LOOP));
                        vb.swap();
                    }

                    MultiGaussian mgsample = new MultiGaussian(samples);
                    {
                        VisWorld.Buffer vb = vw.getBuffer("ellipse-sample");
                        vb.addBack(new VisLines(new VisVertexData(mgsample.getContour(9)),
                                                new VisConstantColor(Color.green), 2,
                                                VisLines.TYPE.LINE_LOOP));
                        vb.swap();

                    }

                    {
                        VisWorld.Buffer vb = vw.getBuffer("samples");
                        vb.addBack(new VisPoints(new VisVertexData(samples),
                                                new VisConstantColor(Color.red),
                                                2));
                        vb.swap();
                    }

                    {
                        VisVertexData vd  = new VisVertexData();
                        vd.add(new double[2]); vd.add(new double[]{pg.gd("rx"), 0});
                        vd.add(new double[2]); vd.add(new double[]{0,pg.gd("ry")});

                        VisWorld.Buffer vb = vw.getBuffer("semiaxes");
                        vb.addBack(new VisChain(LinAlg.translate(mu[0],mu[1],0),
                                                LinAlg.rotateZ(pg.gd("theta")),
                                                new VisLines(vd,new VisConstantColor(Color.blue), 3, VisLines.TYPE.LINES)));
                        vb.swap();

                    }



                }
            });

        JFrame jf = new JFrame("Task3");
        jf.setSize(800,600);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setVisible(true);

        pg.notifyListeners("aaaa");


    }


}