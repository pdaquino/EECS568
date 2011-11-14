package team;

import lcm.logging.*;

import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import javax.swing.*;

import april.vis.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;
import april.lcmtypes.*;

// note that you can
// shift/ctrl click in the left most pane to move the robot positions.
public class Task3 implements ParameterListener
{

    JFrame jf;

    // map display
    VisWorld vwm = new VisWorld();
    VisLayer vlm = new VisLayer(vwm);
    VisCanvas vcm = new VisCanvas(vlm);

    VisWorld vwa = new VisWorld();
    VisLayer vla = new VisLayer(vwa);
    VisCanvas vca = new VisCanvas(vla);

    VisWorld vwb = new VisWorld();
    VisLayer vlb = new VisLayer(vwb);
    VisCanvas vcb = new VisCanvas(vlb);

    ParameterGUI pg = new ParameterGUI();

    Log loga, logb;

    public static void main(String args[])
    {
        if (args.length != 1) {
            System.out.println("Specify a log file as an argument.");
            return;
        }

        try {
            new Task3(args);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }

    public void parameterChanged(ParameterGUI pg, String name)
    {
        update();
    }

    ArrayList<pose_t> allPoses = new ArrayList<pose_t>();
    ArrayList<Double> allPosesPosition = new ArrayList<Double>();

    public Task3(String args[]) throws IOException
    {
        pg.addDoubleSlider("loga_pos", "Position A", 0, 1, 0.0);
        pg.addDoubleSlider("logb_pos", "Position B", 0, 1, 0.0);

        loga = new Log(args[0], "r");
        logb = new Log(args[0], "r");

        try {
            while (true) {
                pose_t p = toPoseMessage(readNextEvent(loga, "POSE"));
                allPoses.add(p);
                allPosesPosition.add(loga.getPositionFraction());
            }
        } catch (IOException ex) {
        }
        System.out.println("Read "+allPoses.size()+" poses");

        loga.seekPositionFraction(0);

        pg.addListener(this);

        jf = new JFrame(this.getClass().getName());
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        JSplitPane jsp = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, vca, vcb);
        jsp.setDividerLocation(0.5);
        jsp.setResizeWeight(0.5);

        JSplitPane jsp2 = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, vcm, jsp);
        jsp2.setDividerLocation(0.33);
        jsp2.setResizeWeight(0.33);

        jf.add(jsp2, BorderLayout.CENTER);
        jf.add(pg.getPanel(), BorderLayout.SOUTH);

        jf.setSize(1024,600);
        jf.setVisible(true);

        vlm.addEventHandler(new MyEventHandler());
        update();
    }

    class MyEventHandler extends VisEventAdapter
    {
        boolean captured = false;

        public int getPriority()
        {
            return 100;
        }

        public boolean mouseClicked(VisCanvas vc,  VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            return handle(vc, ray, e);
        }

        public boolean mouseDragged(VisCanvas vc,  VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            return handle(vc, ray, e);
        }


        public boolean mouseReleased(VisCanvas vc,  VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            if (!captured)
                return false;
            captured = false;
            return true;
        }

        boolean handle(VisCanvas vc, GRay3D ray, MouseEvent e)
        {
            int mods=e.getModifiersEx();
            boolean shift = (mods&MouseEvent.SHIFT_DOWN_MASK)>0;
            boolean ctrl = (mods&MouseEvent.CTRL_DOWN_MASK)>0;

            double bestDist = 2;
            int bestIndex = -1;

            double xy[] = ray.intersectPlaneXY(0);

            for (int i = 0; i < allPoses.size(); i++) {
                pose_t p = allPoses.get(i);
                double dist = LinAlg.distance(xy, p.pos, 2);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIndex = i;
                }
            }

            if (bestIndex < 0)
                return captured;

            if (shift) {
                pg.sd("loga_pos", allPosesPosition.get(bestIndex));
                captured = true;
            }
            if (ctrl) {
                pg.sd("logb_pos", allPosesPosition.get(bestIndex));
                captured = true;
            }

            update();

            return captured;
        }
    }

    Log.Event readNextEvent(Log log, String channel) throws IOException
    {
        while (true) {
            Log.Event ev = log.readNext();
            if (ev.channel.equals(channel))
                return ev;
        }
    }

    laser_t toLaserMessage(Log.Event ev) throws IOException
    {
        ByteArrayInputStream bins = new ByteArrayInputStream(ev.data);
        DataInputStream dins = new DataInputStream(bins);

        return new laser_t(dins);
    }

    pose_t toPoseMessage(Log.Event ev) throws IOException
    {
        ByteArrayInputStream bins = new ByteArrayInputStream(ev.data);
        DataInputStream dins = new DataInputStream(bins);

        return new pose_t(dins);
    }

    ArrayList<double[]> laserToPoints(laser_t laser)
    {
        ArrayList<double[]> points = new ArrayList<double[]>();
        for (int i = 0; i < laser.nranges; i++) {
            double theta = laser.rad0 + laser.radstep*i;
            points.add(new double[] { laser.ranges[i] * Math.cos(theta),
                laser.ranges[i] * Math.sin(theta) });
        }
        return points;
    }

    // Returns an xyt aligning a with b
    private double[] RANSAC(ArrayList<Task2.Line> linesa, ArrayList<Task2.Line> linesb, ArrayList<double[]> pointsa)
    {
        assert(linesa.size() > 2 && linesb.size() > 2);
        //System.out.printf("%d, %d\n", linesa.size(), linesb.size());

        double[] xyt = new double[3];   // "Best model"
        int bestConsensus = -1;

        Random rand = new Random(15897143);
        Tic tic = new Tic();
        for (int i = 0; i < 10000; i++) {
            Task2.Line line1, line2, line3;
            int idx1, idx2, idx3;
            idx1 = rand.nextInt(linesa.size());
            do {
                idx2 = rand.nextInt(linesa.size());
            } while (idx2 == idx1);
            do {
                idx3 = rand.nextInt(linesa.size());
            } while (idx3 == idx1 && idx3 == idx2);
            line1 = linesa.get(idx1);
            line2 = linesa.get(idx2);
            line3 = linesa.get(idx3);

            Task2.Line line1b, line2b, line3b;
            idx1 = rand.nextInt(linesb.size());
            int cnt1 = 0, cnt2 = 0;
            do {
                idx2 = rand.nextInt(linesb.size());
                cnt1++;
            } while (idx2 == idx1);
            do {
                idx3 = rand.nextInt(linesb.size());
                cnt2++;
            } while (idx3 == idx1 && idx3 == idx2);
            line1b = linesb.get(idx1);
            line2b = linesb.get(idx2);
            line3b = linesb.get(idx3);
            //System.out.printf("%d, %d\n", cnt1, cnt2);

            //tic.tic();
            // Compute line intersections and RBT
            ArrayList<double[]> isects = new ArrayList<double[]>();
            ArrayList<double[]> isectsb = new ArrayList<double[]>();
            try {
                isects.add(line1.intersect(line2));
                isects.add(line1.intersect(line3));
                isects.add(line2.intersect(line3));

                isectsb.add(line1b.intersect(line2b));
                isectsb.add(line1b.intersect(line3b));
                isectsb.add(line2b.intersect(line3b));
            } catch (ParallelLineException ple) {
                continue;
            }

            double[] transform = TransformationFit.getTransformation(isects, isectsb);
            //System.out.printf("RBT took %f s\n", tic.toc());

            // Transform the lines
            //tic.tic();
            double[] invXform = LinAlg.xytInverse(transform);
            ArrayList<Task2.Line> xformb = new ArrayList<Task2.Line>();
            for (Task2.Line line: linesb) {
                xformb.add(line.transform(invXform));
            }
            //System.out.printf("Applying xform took %f s\n", tic.toc());

            // Apply RBT to points in A and compute consensus score
            //tic.tic();
            int score = 0;
            double thresh = .1;
            for (double[] pa: pointsa) {
                for (Task2.Line line: xformb) {
                    double[] cpa = line.closestPoint(pa);
                    double temp = LinAlg.distance(cpa, pa);
                    if (temp < thresh) {
                        score++;
                        break;
                    }
                }
            }
            //System.out.printf("Consensus took %f s\n", tic.toc());

            if (score > bestConsensus) {
                bestConsensus = score;
                xyt = transform;
            }
            /*if (bestConsensus >= .7*pointsa.size()) {
                return xyt;
            }*/
        }

        return xyt;
    }

    public void update()
    {
        laser_t lasera, laserb;
        pose_t posea, poseb;

        try {
            loga.seekPositionFraction(pg.gd("loga_pos"));
            logb.seekPositionFraction(pg.gd("logb_pos"));

            lasera = toLaserMessage(readNextEvent(loga, "LIDAR_FRONT"));
            posea  = toPoseMessage(readNextEvent(loga, "POSE"));
            laserb = toLaserMessage(readNextEvent(logb, "LIDAR_FRONT"));
            poseb  = toPoseMessage(readNextEvent(logb, "POSE"));

        } catch (IOException ex) {
            System.out.println("ex: "+ex);
            return;
        }

        ArrayList<double[]> pointsa = laserToPoints(lasera);
        ArrayList<double[]> pointsb = laserToPoints(laserb);

        ///////////////////////////////////////////////////////////////
        // You'll need to add some code here to process the laser data.
        ///////////////////////////////////////////////////////////////
        ArrayList<Task2.Line> linesa = Task2.agglomerateLines(pointsa, 0.03, 200);
        ArrayList<Task2.Line> linesb = Task2.agglomerateLines(pointsb, 0.03, 200);
        double[] xyt = RANSAC(linesa, linesb, pointsa);
        //LinAlg.print(xyt);
        //LinAlg.print(LinAlg.quatToRollPitchYaw(posea.orientation));

        /*
           ArrayList<double[]> pa = new ArrayList<double[]>();
           pa.add(new double[] {0,0});
           pa.add(new double[] {1,1});
           ArrayList<double[]> pb = new ArrayList<double[]>();
           pb.add(new double[] {0,2});
           pb.add(new double[] {2,0});
           Task2.Line linea = new Task2.Line(pa);
           Task2.Line lineb = new Task2.Line(pb);
           linea.intersect(lineb);
           */

        if (true) {
            // left-most panel: draw the odometry path
            VisWorld.Buffer vb = vwm.getBuffer("map");
            ArrayList<double[]> points = new ArrayList<double[]>();
            for (pose_t p : allPoses)
                points.add(p.pos);
            vb.addBack(new VisPoints(new VisVertexData(points),
                                     new VisConstantColor(Color.gray), 2));

            vb.addBack(new VisChain(LinAlg.quatPosToMatrix(posea.orientation, posea.pos),
                                    new VisRobot(Color.blue)));

            vb.addBack(new VisChain(LinAlg.quatPosToMatrix(poseb.orientation, poseb.pos),
                                    new VisRobot(Color.red)));

            vb.swap();
        }

        if (true) {
            // draw middle panel (laser scan a)
            VisWorld.Buffer vb = vwa.getBuffer("points");
            vb.addBack(new VisPoints(new VisVertexData(pointsa),
                                     new VisConstantColor(Color.blue),2));
            vb.swap();
        }

        if (true) {
            // draw right panel (laser scan b)
            VisWorld.Buffer vb = vwb.getBuffer("points");
            vb.addBack(new VisPoints(new VisVertexData(pointsb),
                                     new VisConstantColor(Color.blue),2));
            // Add in red points showing transformation of a points to b coordinates
            vb.addBack(new VisChain(LinAlg.xytToMatrix(xyt),
                                    new VisPoints(new VisVertexData(pointsa),
                                                  new VisConstantColor(Color.red), 1)));
            vb.swap();
        }

    }
}
