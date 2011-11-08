package team;

import javax.swing.*;
import java.awt.*;

import java.util.*;
import java.io.*;

import april.vis.*;
import april.util.*;
import april.lcmtypes.*;

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
    ArrayList<ArrayList<double[]> > lines = new ArrayList<ArrayList<double[]> >();

    public Task2()
    {
        pg.addDoubleSlider("thresh","Thresh",0,1,.5);
        pg.addInt("maxsteps", "Max Agglomeration Steps", 100);

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

        private void computeLineFit()
        {
            assert(line != null && line.size() > 0);
            Mx = My = Mxx = Mxy = Myy = 0;
            for (int i = 0; i < line.size(); i++) {
                double[] xy = line.get(i);
                Mx += xy[0];
                My += xy[1];
                Mxx += xy[0]*xy[0];
                Myy += xy[1]*xy[1];
                Mxy += xy[0]*xy[1];
            }

            // Can optimize later XXX
            double Cxx = Mxx/line.size() - (Mx*Mx)/(line.size()*line.size());
            double Cyy = Myy/line.size() - (My*My)/(line.size()*line.size());
            double Cxy = Mxy/line.size() - (Mx*My)/(line.size()*line.size());

            q = new double[] {Mx/line.size(), My/line.size()};
            theta = MathUtil.mod2pi(Math.PI + .5*MathUtil.atan2(-2*Cxy, Cyy-Cxx));
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
        public ArrayList<double[]> getLine()
        {
            return null;
        }
    }

    // Get lines from a scan using agglomeration
    public static ArrayList<Line> > agglomerateLines(ArrayList<double[]> points)
    {
        // Initialize tiny lines
        ArrayList<Line> lines = new ArrayList<Line>();
        for (int i = 0; i < points.size()-1; i++) {
            ArrayList<double[]> line = new ArrayList<double[]>();
            line.add(points.get(i));
            line.add(points.get(i+1));
            lines.add(new Line(line));
        }

        boolean done = false;
        while (!done) {
            done = true;
            for (int i = 0; i < lines.size()-1; i++) {

            }
        }

    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("LIDAR_FRONT")) {
                laser = new laser_t(ins);

                lines = agglomerateLines(laserToPoints(laser));

                update();
            } else if (channel.equals("POSE")) {
                // do nothing for Task 2
            }
        } catch(IOException e) {
            System.out.println("Failed to decode message on channel "+channel);
        }
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

    public static void main(String args[])
    {
        new Task2();
    }


}
