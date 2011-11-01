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
            vb.swap();
        }

    }
}
