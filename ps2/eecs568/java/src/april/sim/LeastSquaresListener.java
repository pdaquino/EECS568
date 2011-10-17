package april.sim;

import java.awt.*;
import java.util.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;
import team.*;

public class LeastSquaresListener implements Simulator.Listener {

    VisWorld vw;
    Config config;
    double baseline; // Robot baseline in [m]
    
    ArrayList<Node> nodes = new ArrayList<Node>();
    ArrayList<Edge> edges = new ArrayList<Edge>();
    
    private RobotPose latestRobotPose;
    
    double xyt[] = new double[3]; // dead reconning
    ArrayList<double[]> trajectory = new ArrayList<double[]>();

    public void init(Config config_, VisWorld vw_) {
        config = config_;
        vw = vw_;

        baseline = config.requireDouble("robot.baseline_m");
        
        latestRobotPose = new RobotPose(0);
        latestRobotPose.setPosition(new double[]{0,0,0});
        nodes.add(latestRobotPose);
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets) {
        // Magic stuff here
        // build J
        // newRobotPose will be updated by the constructor of OdometryEdge
        RobotPose newRobotPose = new RobotPose(nodes.size());
        OdometryEdge odomEdge = new OdometryEdge(odom.obs[0], odom.obs[1],
                baseline, latestRobotPose, newRobotPose);
        nodes.add(newRobotPose);
        edges.add(odomEdge);
        this.latestRobotPose = newRobotPose;
        
        Matrix J = buildJacobian();
        double[] r = buildResidual();
        
        // Draw our trajectory and detections
        drawStuff();
    }

    public void drawStuff() {
        // Draw stuff here
    }

    public void drawDummy(ArrayList<Simulator.landmark_t> landmarks) {
        // Draw local Trajectory
        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
            vb.addBack(new VisLines(new VisVertexData(trajectory),
                    new VisConstantColor(new Color(160, 30, 30)),
                    1.5, VisLines.TYPE.LINE_STRIP));
            vb.swap();
        }

        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        rpoints.add(new double[]{-.3, .3});
        rpoints.add(new double[]{-.3, -.3});
        rpoints.add(new double[]{.45, 0});

        // Probably should be replaced with student-code
        {
            VisWorld.Buffer vb = vw.getBuffer("robot-local");
            VisObject robot = new VisLines(new VisVertexData(rpoints),
                    new VisConstantColor(Color.red),
                    3,
                    VisLines.TYPE.LINE_LOOP);


            double xyzrpy[] = new double[]{xyt[0], xyt[1], 0,
                0, 0, xyt[2]};
            vb.addBack(new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), robot));
            vb.swap();
        }

        // Draw the landmark observations
        {
            VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
            for (Simulator.landmark_t lmark : landmarks) {
                double[] obs = lmark.obs;
                ArrayList<double[]> obsPoints = new ArrayList<double[]>();
                obsPoints.add(LinAlg.resize(xyt, 2)); Matrix p;
                double rel_xy[] = {obs[0] * Math.cos(obs[1]), obs[0] * Math.sin(obs[1])};
                obsPoints.add(LinAlg.transform(xyt, rel_xy));
                vb.addBack(new VisLines(new VisVertexData(obsPoints),
                        new VisConstantColor(lmark.id == -1 ? Color.gray : Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
            }
            vb.swap();
        }
    }

    private Matrix buildJacobian() {
        int numColumns = getStateVectorSize();
        int numRows = getNumJacobianRows();
        Matrix J = new Matrix(numRows, numColumns, Matrix.SPARSE);
        int currentRow = 0;
        for(Edge e : edges) {
            CSRVec[] edgeRows = e.getJacobianRows(numColumns);
            for(CSRVec row : edgeRows) {
                J.setRow(currentRow, row);
                currentRow++;
            }
        }
        assert J.getColumnDimension() == numColumns;
        assert J.getRowDimension() == numRows;
        return J;
    }

    private int getStateVectorSize() {
        int size = 0;
        for(Node n : nodes) {
            size += n.getNumDimensions();
        }
        return size;
    }
    
    private int getNumJacobianRows() {
        int size = 0;
        for(Edge e : edges) {
            size += e.getNumberJacobianRows();
        }
        return size;
    }

    private double[] buildResidual() {
        double[] residual = new double[getStateVectorSize()];
        int currentRow = 0;
        for(Edge e : edges) {
            double[] edgeResidual = e.getResidual();
            for(double r : edgeResidual) {
                residual[currentRow] = r;
                currentRow++;
            }
        }
        return residual;
    }
}
