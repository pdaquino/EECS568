package team;

import javax.swing.*;
import java.awt.*;

import java.util.*;
import java.io.*;

import april.vis.*;
import april.util.*;
import april.lcmtypes.*;
import april.jmat.*;
import april.jmat.geom.*;

import lcm.lcm.*;


public class Task2 implements LCMSubscriber, ParameterListener
{

    static LCM lcm = LCM.getSingleton();

    JFrame jf = new JFrame("Task2");

    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    ParameterGUI pg = new ParameterGUI();

    laser_t laser; // synchronize on 'this'
    ArrayList<Line> lines = new ArrayList<Line>();

    public Task2()
    {
        pg.addDoubleSlider("thresh","Thresh",0,1,.05);
        pg.addInt("maxsteps", "Max Agglomeration Steps", 200);

        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        jf.setSize(800,600);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        lcm.subscribe("LIDAR_FRONT",this);
        lcm.subscribe("POSE",this);

    }

    public static class Line
    {
        ArrayList<double[]> line;

        // Line fit
        double[] q;
        double theta;

        // Moments
        double Mx, My, Mxx, Mxy, Myy;

        public Line(ArrayList<double[]> line_)
        {
            line = line_;
            computeLineFit();
        }

        public int size()
        {
            return line.size();
        }

        private void computeLineFit()
        {
            assert(line != null && line.size() > 0);

            Mx = 0;
            My = 0;
            Mxx = 0;
            Mxy = 0;
            Myy = 0;
            for (int i = 0; i < line.size(); i++) {
                double[] xy = line.get(i);
                Mx += xy[0];
                My += xy[1];
                Mxx += xy[0]*xy[0];
                Myy += xy[1]*xy[1];
                Mxy += xy[0]*xy[1];
            }

            // Can optimize later XXX
            double Cxx = (Mxx/line.size()) - ((Mx*Mx)/(line.size()*line.size()));
            double Cyy = (Myy/line.size()) - ((My*My)/(line.size()*line.size()));
            double Cxy = (Mxy/line.size()) - ((Mx*My)/(line.size()*line.size()));

            //System.out.printf("%f %f %f %f %f === %f %f %f\n", Mx, My, Mxx, Myy, Mxy, Cxx, Cyy, Cxy);

            q = new double[] {Mx/line.size(), My/line.size()};
            //LinAlg.print(q);
            theta = MathUtil.mod2pi(Math.PI/2 + MathUtil.atan2(-2*Cxy, Cyy-Cxx)/2);
            //System.out.printf("theta: %f\n", theta);
        }

        // If the merged line is below our threshold, keep it. Otherwise, return null
        // Assumes Line l will share its first point with your last
        public Line tryLineMerge(Line l, double threshold)
        {
            // Check for adjacency
            assert(Arrays.equals(line.get(line.size()-1), l.line.get(0)));

            ArrayList<double[]> points = new ArrayList<double[]>();
            points.addAll(line);
            points.remove(points.size()-1);
            points.addAll(l.line);
            Line temp = new Line(points);

            if (temp.getError() <= threshold)
                return temp;
            else
                return null;
        }

        public double getError()
        {
            double ct = Math.cos(theta);
            double st = Math.sin(theta);

            return ct*ct*(Myy - (My*My)/line.size()) -
                   2*ct*st*(Mxy - (Mx*My)/line.size()) +
                   st*st*(Mxx - (Mx*Mx)/line.size());
        }

        // Get a line for drawing purposes XXX
        public ArrayList<double[]> getLineSeg()
        {
            ArrayList<double[]> lineSegment = new ArrayList<double[]>();

            // Calculate endpoints
            lineSegment.add(closestPoint(line.get(0)));
            lineSegment.add(closestPoint(line.get(line.size()-1)));

            return lineSegment;
        }

        public double[] getLine()
        {
            return new double[] {q[0], q[1], theta};
        }

        public double[] closestPoint(double[] xy)
        {
            // Calculate endpoints
            double ct = Math.cos(theta);
            double st = Math.sin(theta);
            double[] n = LinAlg.normalize(new double[] {ct, st});
            double[] v = LinAlg.normalize(new double[] {-st, ct});

            double dotqv = LinAlg.dotProduct(q, v);
            double[] q1 = new double[] {v[0]*dotqv, v[1]*dotqv};

            double dotnxy = LinAlg.dotProduct(xy, n);
            return new double[] {n[0]*dotnxy + q1[0],
                                 n[1]*dotnxy + q1[1]};
        }

        public double[] intersect(Line b)
        {
            // Parallel with same theta
            if (MathUtil.doubleEquals(MathUtil.mod2pi(theta),
                                      MathUtil.mod2pi(b.theta)))
                return null;

            double[] cp0 = closestPoint(new double[2]);
            double[] cp1 = b.closestPoint(new double[2]);
            double r0 = LinAlg.magnitude(cp0);
            double r1 = LinAlg.magnitude(cp1);

            double t0 = MathUtil.atan2(cp0[1], cp0[0]);
            double t1 = MathUtil.atan2(cp1[1], cp1[0]);
            double ct0 = Math.cos(t0);
            double st0 = Math.sin(t0);
            double ct1 = Math.cos(t1);
            double st1 = Math.sin(t1);
            double[][] a = new double[][] {{ct0, st0},
                                           {ct1, st1}};
            Matrix A = new Matrix(a);
            double[] xy = A.inverse().times(new double[] {r0, r1});
            //LinAlg.print(xy);

            return xy;
        }
    }

    // Get lines from a scan using agglomeration
    static public ArrayList<Line> agglomerateLines(ArrayList<double[]> points,
                                                   double threshold,
                                                   int maxSteps)
    {
        // Initialize tiny lines
        ArrayList<Line> lines = new ArrayList<Line>();
        for (int i = 0; i < points.size()-1; i++) {
            ArrayList<double[]> line = new ArrayList<double[]>();
            line.add(points.get(i));
            line.add(points.get(i+1));
            lines.add(new Line(line));
        }

        // Try to merge lines until none are less than our minimum error cost
        for (int iters = 0; iters < maxSteps; iters++) {
            Line best = null;
            double err = Double.MAX_VALUE;
            int idx = -1;
            for (int i = 0; i < lines.size()-1; i++) {
                // XXX Priority queues are the shit
                Line temp = lines.get(i).tryLineMerge(lines.get(i+1), threshold);
                if (temp != null && temp.getError() < err) {
                    best = temp;
                    err = temp.getError();
                    idx = i;
                }
            }
            if (best == null)
                break;

            // Remove old lines and add in new
            lines.set(idx, best);
            lines.remove(idx+1);
        }

        return lines;
    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("LIDAR_FRONT")) {
                laser = new laser_t(ins);

                lines = agglomerateLines(laserToPoints(laser),
                                         pg.gd("thresh"),
                                         pg.gi("maxsteps"));
                // Strip out undersized lines
                pruneLines();

                update();
            } else if (channel.equals("POSE")) {
                // do nothing for Task 2
            }
        } catch(IOException e) {
            System.out.println("Failed to decode message on channel "+channel);
        }
    }

    private void pruneLines()
    {
        ArrayList<Line> newLines = new ArrayList<Line>();
        for (Line l: lines) {
            if (l.size() > 4) {
                newLines.add(l);
            }
        }
        lines = newLines;
    }

    public synchronized void update()
    {
        {
            VisWorld.Buffer vb = vw.getBuffer("laser-points");
            vb.addBack(new VisPoints(new VisVertexData(laserToPoints(laser)),
                                     new VisConstantColor(Color.green),
                                     2));
                    vb.swap();
        }

        // Draw the lines
        {
            VisWorld.Buffer vb = vw.getBuffer("lines");
            for (Line l: lines) {
                ArrayList<double[]> points = l.getLineSeg();
                if (points == null)
                    continue;
                VisConstantColor vcc = new VisConstantColor(randomColor());
                VisVertexData vvd = new VisVertexData(points);
                vb.addBack(new VisLines(vvd,
                                        vcc,
                                        1.0,
                                        VisLines.TYPE.LINES));
            }
            vb.swap();
        }
    }

    public void parameterChanged(ParameterGUI pg, String name)
    {
        if (name.equals("thresh"))
            update();
    }

    public static ArrayList<double[]> laserToPoints(laser_t laser)
    {
        ArrayList<double[]> points = new ArrayList<double[]>();
        for (int i = 0; i < laser.nranges; i++) {
            double theta = laser.rad0 + laser.radstep*i;
            points.add(new double[] { laser.ranges[i] * Math.cos(theta),
                                      laser.ranges[i] * Math.sin(theta) });
        }
        return points;
    }

    static Random r = new Random();
    public static Color randomColor()
    {
        return new Color(r.nextInt(255), r.nextInt(255), r.nextInt(255));
    }

    public static void main(String args[])
    {
        new Task2();
    }


}
