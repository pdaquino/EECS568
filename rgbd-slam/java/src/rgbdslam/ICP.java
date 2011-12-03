package rgbdslam;

import kinect.*;

import april.jmat.*;
import april.jmat.geom.*;

import java.lang.Double;
import java.util.ArrayList;

public class ICP
{
    // XXX No clue if these are the right values
    final static int MAX_ITERATIONS = 50; // maximum number of iteration for ICP
    final static double THRESHOLD = 0.1; // threshold for convergence change in normalized error
    final static double DISCARD_D = 10; // threshold for outlier rejection
    
    private KdTree.SqrEuclid<double[]> kdtree; // kdtree for storing points in B
    
    // Constructs ICP iterator which can be called to allign ColorPointClouds with this one
    public ICP(ColorPointCloud cpcB) {
        
        ArrayList<double[]> PB = cpcB.points;
        
        // XXX downsample paper claimed downsampling was good
        
        if (PB.size() > 0) {
           kdtree = new KdTree.SqrEuclid<double[]>(3,PB.size()); // points are in 3 space
           // add points
           for (double[] P: PB) {
               kdtree.addPoint(P,P); // since our points are simply points in 3D space
               // we are sorting based on their location in 3D space and the point is what we want back
           }
        } else {
            kdtree = new KdTree.SqrEuclid<double[]>(3,0); // max size of 0
        } 
    }
    
    // XXX alternate constructor that uses our Voxel represenation for B ?
    
    // refines initial estimate of an RBT that alligns points in A with points in B
    // in frame B
    public double[][] match(ColorPointCloud cpcA, double[][] rbt) {
        int cntr = 0;
        double curerror = 0; // these values will represent an average error
        double preverror = Double.MAX_VALUE;
        
        // get points stored in colored point clouds
        ArrayList<double[]> PA = cpcA.points;
        
        // XXX downsample? paper claimed downsampling was good
                
        // itterate until change in error becomes small or reach max iterations
        while (((preverror - curerror) > THRESHOLD) && (cntr < MAX_ITERATIONS)) {
            // apply rbt to points in A to get into frame B
            ArrayList<double[]> PAinB = LinAlg.transform(rbt,PA);
            
            // good correspondance points
            ArrayList<double[]> GoodA = new ArrayList<double[]>();
            ArrayList<double[]> GoodB = new ArrayList<double[]>();
            
            double totalError = 0; // for accumulating error
            
            // for each transformed point
            for (double[] A : PAinB) {
            
                // find nearest point in B for each in A using K-D tree
                double[] B = kdtree.nearestNeighbor(A,1,false).get(0);
            
                // compute distance between A and B
                double d = LinAlg.distance(A,B);
                
                // if distance is small enough, then not an outlier
                // need this to handle parts that will never overlap
                if (d < DISCARD_D) {
                    // add both to our list of good points
                    GoodA.add(A);
                    GoodB.add(B);
                    // add distance to our error estimate
                    totalError = totalError + d;
                }    
            }
            
            // reassign errors
            preverror = curerror;
            curerror = totalError/GoodA.size(); // maintaining an average error
            
            // use these lists to compute updated RBT
            rbt = AlignPoints3D.align(cora, corb);
            
            cntr++;
        } 
        
        return rbt;
    }
    
            
}
