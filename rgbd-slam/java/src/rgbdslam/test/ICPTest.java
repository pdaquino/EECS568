package rgbdslam.test;

import kinect.*;

import java.util.*;
import java.awt.event.*;

import april.util.*;
import april.jmat.*;

import rgbdslam.*;

// tests ICP using real kinect point clouds prints estimate of 
// rotation transformation between every pair
public class ICPTest {

    Kinect kinect = new Kinect();
    KinectThread kt;
    ICPThread icpt;

    public ICPTest() {
        kt = new KinectThread();
        kt.start();
        icpt = new ICPThread();
        icpt.start();
    }

    class ICPThread extends Thread {

        ColorPointCloud prevCPC;
        ColorPointCloud curCPC;
        double[][] RBT;
        ICP icp;
        double[][] CRBT; // current cumulative orientation

        public ICPThread() {
            prevCPC = null;
            curCPC = null;
            RBT = new double[][]{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
            icp = null;
            CRBT = RBT;
        }

        synchronized public void run() {
            while (true) {
                if ((this.curCPC != null) && (this.prevCPC != null)) {
                    resetRBT();
                    
                    double timeInit = 0;
                    double timeMatch = 0;
                    
                    System.out.println("Attempting to match " 
                            + curCPC.points.size() + " points.");
                    
                    
                    // the actual test!
                    Tic tic = new Tic();
                    icp = new ICP(prevCPC);
                    timeInit = tic.toc();
                    tic = new Tic();
                    RBT = icp.match(curCPC, RBT);
                    timeMatch = tic.toc();

                    
                    System.out.print("\n");
                    System.out.println("KD construction time " + timeInit);
                    System.out.println("Matching time " + timeMatch);
                    CRBT = LinAlg.matrixAB(CRBT, RBT);
                    LinAlg.print(RBT);
                    System.out.println();
                    LinAlg.print(CRBT);
                }

                try {
                    wait(); // go to sleep until we get another update
                } catch (InterruptedException ex) {
                }
            }
        }

        // reset RBT estimate to identity
        synchronized private void resetRBT() {
            /*
            RBT = new double[][] 
                {{0.9848, -0.1736, 0, 0}, 
                {0.1736, 0.9848, 0, 0}, 
                {0, 0, 1, 0}, 
                {0, 0, 0, 1}}; */
            RBT = new double[][]{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        }

        synchronized public void accumulateFrame(Kinect.Frame frame) {
            if (this.curCPC == null) {
                ColorPointCloud pointCloud = new ColorPointCloud(frame,10);
                this.curCPC = pointCloud;
                this.prevCPC = pointCloud;
            } else {
                this.prevCPC = curCPC;
                this.curCPC = new ColorPointCloud(frame,10); // need to play with this decimation factor
            }
            notify();
        }
    }

    class KinectThread extends Thread {

        int fps = 2;
        boolean closeFlag;

        public KinectThread() {
            closeFlag = false;
        }

        public void run() {
            System.out.println("Starting kinect thread");
            kinect.init();
            kinect.start();

            while (!closeFlag) {
                Kinect.Frame f = kinect.getFrame();


                if (f != null) {
                    icpt.accumulateFrame(f);
                }
                TimeUtil.sleep(1000 / fps);
            }

            System.out.println("Buh-bye, now!");
        }

        public void close() {
            closeFlag = true;
            kinect.stop();
            kinect.close();
        }
    }

    static public void main(String[] args) {
        ICPTest test = new ICPTest();
    }
}
