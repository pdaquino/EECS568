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

    private final int NOISE_WAIT = 25;
    private int noiseCount = 0;

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

        edges.add(new ConstraintEdge(latestRobotPose, new double[3]));
    }

    public void update(Simulator.odometry_t odom, ArrayList<Simulator.landmark_t> dets) {
        // Magic stuff here
        // build J
        // newRobotPose will be updated by the constructor of OdometryEdge
        RobotPose newRobotPose = new RobotPose(currentStateVectorSize);
        OdometryEdge odomEdge = new OdometryEdge(config, odom.obs[0], odom.obs[1],
                baseline, latestRobotPose, newRobotPose);
        nodes.add(newRobotPose);
        currentStateVectorSize += newRobotPose.getNumDimensions();
        edges.add(odomEdge);
        this.latestRobotPose = newRobotPose;

        // Deal with landmarks
        ArrayList<LandmarkPose> recentLmarks = new ArrayList<LandmarkPose>();
        /*for (Simulator.landmark_t landmark: dets) {
            LandmarkPose lpose;
            if (lmarks.containsKey(landmark.id)) {
                lpose = lmarks.get(landmark.id);
            } else {
                lpose = new LandmarkPose(currentStateVectorSize);
                currentStateVectorSize += lpose.getNumDimensions();
                double[] robotPos = newRobotPose.getPosition();
                double[] rel_xy = new double[] {landmark.obs[0] * Math.cos(landmark.obs[1]),
                                                landmark.obs[0] * Math.sin(landmark.obs[1])};
                double[] lmarkPos = LinAlg.transform(robotPos, rel_xy);
                lpose.setPosition(lmarkPos);
                nodes.add(lpose);
                lmarks.put(landmark.id, lpose);
            }
            recentLmarks.add(lpose);
            LandmarkEdge ledge = new LandmarkEdge(config, landmark.obs[0], landmark.obs[1], newRobotPose, lpose);
            edges.add(ledge);
        }*/

        //Matrix J = buildJacobian();
        //dumpMatrixDimensions("J",J);
        //J.print();
        //Matrix JT = J.transpose();
        //dumpMatrixDimensions("Jt",JT);
        //Matrix JTJ = JT.times(J);
        //JTJ.print();

        Matrix[] matrices = buildJTSigmaJ();

        // Tikhonov regularlization
        double alpha = 5.0; // regularization constant...XXX choose wisely
        Matrix I = Matrix.identity(currentStateVectorSize, currentStateVectorSize).times(alpha);
        Matrix Tikhonov = matrices[0].plus(I);

        //CholeskyDecomposition solver = new CholeskyDecomposition(JTJ);
        //Matrix updatedState = solver.solve(JTr);
        //LUDecomposition luSolver = new LUDecomposition(JTJ);
        //Matrix updatedState = luSolver.solve(JTr);
        //
        // tikhonov stuff
        /*Matrix I = Matrix.identity(currentStateVectorSize, currentStateVectorSize).times(100);
        LUDecomposition luSolver = new LUDecomposition(JTJ.plus(I));
        Matrix updatedState = luSolver.solve(JTr);
        Matrix deltaX = JTJ.plus(I).inverse().times(JT).times(r);*/

        //XXX Debug
        /*if (noiseCount % NOISE_WAIT == 0) {
            noiseCount = 0;
            addNoise(); // Noise up the state vector and see if we recover
        }
        noiseCount++;*/

        double MAX_ITERS = 100;
        for (int iter = 0; iter < MAX_ITERS; iter++) {
            double[] r = buildResidual();
            //System.out.println(Arrays.toString(r));
            //double[] JTr = JT.times(r); // XXX
            double[] JTSigmar = matrices[1].times(r);

            CholeskyDecomposition solver = new CholeskyDecomposition(Tikhonov);
            double[] deltaX = solver.solve(Matrix.columnMatrix(JTSigmar)).copyAsVector();
            //double[] deltaX = Tikhonov.inverse().times(JTr);

            // Draw our trajectory and detections
            double[] stateVector = getStateVector();
            for(int i = 0; i < stateVector.length; i++) {
                //stateVector[i] += deltaX.get(i, 0);
                stateVector[i] += deltaX[i];
            }
            updateNodesPosition(stateVector);
        }
        MSE(buildResidual());

        xyt = newRobotPose.getPosition();
        drawStuff(recentLmarks);
    }


    // Debugging...assumes no landmarks
    private void addNoise()
    {
        double[] stateVector = getStateVector();

        Random rand = new Random();
        double mag = 0.5;
        for (int i = 0; i < stateVector.length; i++) {
            if (i % 3 == 2)
                continue;
            double delta = rand.nextDouble() - 0.5;
            delta *= mag;
            stateVector[i] += delta;
        }

        updateNodesPosition(stateVector);
    }


    private void dumpMatrixDimensions(String name, Matrix J) {
        System.out.println(name+" is " + J.getRowDimension() + "x" + J.getColumnDimension());
    }

    public void drawStuff(ArrayList<LandmarkPose> recentLmarks) {
        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        rpoints.add(new double[]{-.3, .3});
        rpoints.add(new double[]{-.3, -.3});
        rpoints.add(new double[]{.45, 0});

        trajectory.clear();

        // Draw estimated trajectory based on our state
        for(Node n : nodes) {
            if(!(n instanceof RobotPose)) continue;
            trajectory.add(LinAlg.resize(n.getPosition(), 2));
        }
        System.out.print("Predicted: ");dumpPose(xyt);

        {
            VisWorld.Buffer vb = vw.getBuffer("trajectory-local");
            vb.addBack(new VisLines(new VisVertexData(trajectory),
                    new VisConstantColor(new Color(160, 30, 30)),
                    1.5, VisLines.TYPE.LINE_STRIP));
            vb.swap();
        }

        // Draw estimated robot position
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

        // Draw landmark observations from this run
        {
            VisWorld.Buffer vb = vw.getBuffer("landmarks-noisy");
            for (LandmarkPose lmark : recentLmarks) {
                double[] xy = lmark.getPosition();
                ArrayList<double[]> obsPoints = new ArrayList<double[]>();
                obsPoints.add(LinAlg.resize(xyt, 2));
                obsPoints.add(xy);
                vb.addBack(new VisLines(new VisVertexData(obsPoints),
                        new VisConstantColor(Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
                        //new VisConstantColor(lmark.id == -1 ? Color.gray : Color.cyan), 2, VisLines.TYPE.LINE_STRIP));
            }
            vb.swap();
        }

        // Draw current esimated landmark positions
        {
            VisWorld.Buffer vb = vw.getBuffer("estimated-landmarks");
            ArrayList<double[]> points = new ArrayList<double[]>();
            for (LandmarkPose lmark: lmarks.values()) {
                points.add(lmark.getPosition());
            }
            vb.addBack(new VisPoints(new VisVertexData(points),
                                     new VisConstantColor(Color.red),
                                     5));
            vb.swap();
        }
    }

    private void dumpPose(double[] xyt) {
        System.out.println("("+xyt[0]+","+xyt[1]+","+xyt[2]+")");
    }

    /*private Matrix buildJacobian() {
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
    }*/

    private Matrix[] buildJTSigmaJ() {
        int numStates = getStateVectorSize();
        int numRows = getNumJacobianRows();
        Matrix sum0 = new Matrix(numStates, numStates, Matrix.SPARSE);  // JTSigmaJ
        Matrix sum1 = new Matrix(numStates, numRows, Matrix.SPARSE);    // JTSigma

        int columnCnt = 0;
        for (Edge e: edges) {
            // Get J rows...(Ji) as a matrix
            Matrix J = e.getJacobian(numStates);
            //J.print();
            // Transpose it
            Matrix JT = J.transpose();
            // Get covariance matrix
            Matrix Sigma = e.getCovarianceInverse();
            //Sigma.print();
            // sum.plus(Ji'*sigma^-1*Ji)
            Matrix JTSigma = JT.times(Sigma);
            sum0.plusEquals(JTSigma.times(J));
            sum1.plusEquals(0, columnCnt, JTSigma);
            columnCnt += e.getNumberJacobianRows();
        }

        Matrix[] sums = new Matrix[] {sum0, sum1};

        return sums;
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

    private double[] buildResidual() {
        //Matrix residual = new Matrix(getNumJacobianRows(), 1, Matrix.DENSE);
        double[] residual = new double[getNumJacobianRows()];
        int currentRow = 0;
        double mse = 0;
        for(Edge e : edges) {
            double[] edgeResidual = e.getResidual();
            for(double r : edgeResidual) {
                //residual.set(currentRow++, 0, r);
                residual[currentRow++] = r;
                mse += r*r;
            }
        }
        // Output mean square error
        //System.out.printf("MSE: %f\n", mse);
        // Chi^2 error?
        return residual;
    }

    private double[] getStateVector() {
        double[] stateVector = new double[getStateVectorSize()];
        for(Node n : nodes) {
            double[] position = n.getPosition();
            int idx = n.getIndex();
            for(int i = 0; i < position.length; i++) {
                stateVector[idx + i] = position[i];
            }
        }
        return stateVector;
    }

    private void updateNodesPosition(double[] updatedStateVector) {
        for(Node n : nodes) {
            double[] position = new double[n.getNumDimensions()];
            int idx = n.getIndex();
            for(int i = 0; i < position.length; i++) {
                position[i] = updatedStateVector[idx + i];
            }
            n.setPosition(position);
        }
    }

    private void MSE(double[] r)
    {
        double mse = 0;
        for (double err: r)
            mse += err*err;
        System.out.printf("MSE: %f\n", mse);
    }
}
