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

    HashMap<Integer, LandmarkPose> lmarks = new HashMap<Integer, LandmarkPose>();

    private RobotPose latestRobotPose;
    private int currentStateVectorSize = 0;

    double xyt[] = new double[3]; // Best guess of our most recent pose
    ArrayList<double[]> trajectory = new ArrayList<double[]>();

    public void init(Config config_, VisWorld vw_) {
        config = config_;
        vw = vw_;

        baseline = config.requireDouble("robot.baseline_m");

        latestRobotPose = new RobotPose(0);
        latestRobotPose.setPosition(new double[]{0,0,0});
        nodes.add(latestRobotPose);
        currentStateVectorSize += latestRobotPose.getNumDimensions();

        edges.add(new ConstraintEdge());
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets) {
        // Magic stuff here
        // build J
        // newRobotPose will be updated by the constructor of OdometryEdge
        RobotPose newRobotPose = new RobotPose(currentStateVectorSize);
        OdometryEdge odomEdge = new OdometryEdge(odom.obs[0], odom.obs[1],
                baseline, latestRobotPose, newRobotPose);
        nodes.add(newRobotPose);
        currentStateVectorSize += newRobotPose.getNumDimensions();
        edges.add(odomEdge);
        this.latestRobotPose = newRobotPose;

        // Deal with landmarks
        for (Simulator.landmark_t landmark: dets) {
            LandmarkPose lpose;
            if (lmarks.containsKey(landmark.id)) {
                lpose = lmarks.get(landmark.id);
            } else {
                lpose = new LandmarkPose(currentStateVectorSize);
                currentStateVectorSize += lpose.getNumDimensions();
                double[] robotPos = newRobotPose.getPosition();
                double[] lmarkPos = new double[2];
                double theta = MathUtil.mod2pi(robotPos[2] + landmark.obs[1]);
                lmarkPos[0] = robotPos[0] + landmark.obs[0]*Math.cos(theta);
                lmarkPos[1] = robotPos[1] + landmark.obs[1]*Math.sin(theta);
                lpose.setPosition(lmarkPos);
                nodes.add(lpose);
                lmarks.put(landmark.id, lpose);
            }
            LandmarkEdge ledge = new LandmarkEdge(landmark.obs[0], landmark.obs[1], newRobotPose, lpose);
            edges.add(ledge);
        }

        Matrix J = buildJacobian();
        Matrix r = buildResidual();
        dumpMatrixDimensions("J",J);
        //J.print();
        dumpMatrixDimensions("r", r);
        //r.print();
        Matrix JT = J.transpose();
        dumpMatrixDimensions("Jt",JT);
        Matrix JTJ = JT.times(J);
        Matrix JTr = JT.times(r);
        //JTJ.print();
        //CholeskyDecomposition solver = new CholeskyDecomposition(JTJ);
        //Matrix updatedState = solver.solve(JTr);
        //LUDecomposition luSolver = new LUDecomposition(JTJ);
        //Matrix updatedState = luSolver.solve(JTr);
        // tikhonov stuff
        Matrix I = Matrix.identity(currentStateVectorSize, currentStateVectorSize).times(42);
        LUDecomposition luSolver = new LUDecomposition(JTJ.plus(I));
        Matrix updatedState = luSolver.solve(JTr);
        Matrix deltaX = JTJ.plus(I).inverse().times(JT).times(r);
        //dumpMatrixDimensions("Xhat", x_hat);
        //deltaX.print();
        //updatedState.print();
        // Draw our trajectory and detections
        double[] stateVector = getStateVector();
        for(int i = 0; i < stateVector.length; i++) {
            stateVector[i] += deltaX.get(i);
            //System.out.println("i: " + stateVector[i]);
        }
        updateNodesPosition(stateVector);

        xyt = newRobotPose.getPosition();
        drawStuff(dets);
    }

    private void dumpMatrixDimensions(String name, Matrix J) {
        System.out.println(name+" is " + J.getRowDimension() + "x" + J.getColumnDimension());
    }

    public void drawStuff( ArrayList<Simulator.landmark_t> dets) {
        trajectory.clear();
//        double[] currentXyt = new double[3];
        int i = 0;
        //xyt = new double[]{0,0,0};
        for(Node n : nodes) {
            //System.out.print("Node " + (i++) + ": ");
            //dumpPose(n.getPosition());
            if(!(n instanceof RobotPose)) continue;
            //xyt = LinAlg.xytMultiply(xyt, n.getPosition());
            //trajectory.add(LinAlg.resize(xyt,2));
            trajectory.add(LinAlg.resize(n.getPosition(), 2));
        }
        //System.out.print("Last node: ");dumpPose(nodes.get(nodes.size()-1).getPosition());
        System.out.print("Predicted: ");dumpPose(xyt);
        drawDummy(dets);
    }

    private void dumpPose(double[] xyt) {
        System.out.println("("+xyt[0]+","+xyt[1]+","+xyt[2]+")");
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
        return currentStateVectorSize;
    }

    private int getNumJacobianRows() {
        int size = 0;
        for(Edge e : edges) {
            size += e.getNumberJacobianRows();
        }
        return size;
    }

    private Matrix buildResidual() {
        Matrix residual = new Matrix(getNumJacobianRows(), 1, Matrix.DENSE);
        int currentRow = 0;
        for(Edge e : edges) {
            double[] edgeResidual = e.getResidual();
            for(double r : edgeResidual) {
                residual.set(currentRow, 0, r);
                currentRow++;
            }
        }
        return residual;
    }

    private double[] getStateVector() {
        double[] stateVector = new double[getStateVectorSize()];
        int i = 0;
        for(Node n : nodes) {
            double[] position = n.getPosition();
            for(double dimension : position) {
                stateVector[i] = dimension;
                i++;
            }
        }
        return stateVector;
    }

    private void updateNodesPosition(double[] updatedStateVector) {
        int i = 0;
        for(Node n : nodes) {
            double[] position = new double[n.getNumDimensions()];
            for(int j = 0; j < position.length; j++) {
                position[j] = updatedStateVector[i];
                i++;
            }
            n.setPosition(position);
        }
    }
}
