package rgbdslam.test;

import java.util.Collections;
import java.util.List;
import java.util.Random;
import rgbdslam.KdTree.Entry;
import rgbdslam.KdTree.SqrEuclid;

/**
 *
 * @author pdaquino
 */
public class KdTreeTest {
    private static class Point {
        double[] xy;
        public Point(double x, double y) {
            this.xy = new double[] { x, y };
        }
    }
    private static Random rand = new Random(1723687);
    public static void main(String[] args) {
        // there are several versions of kd-tree available. the difference
        // is the distance metric used. for instance, we can choose weighted
        // squared euclidian distance, or Manhattan distance. right now we'll
        // choose unweighted euclidian distance.
        //
        // the first argument in the constructor is the number of dimensions of
        // the points; the second is the maximum number of points in the tree
        // (if this number is exceeded, the oldest points are thrown away). you
        // can pass null to allow for as many points as you wish.
        //
        // in this implementation of kd-tree, each point is comprised of a location,
        // which is a double array whose dimension is provided in the constructor,
        // and an associated value of type T (kd-tree is templated on T). 
        // as far as I can see, the associated value is not actually used to
        // compute the nearest neighbor.
        SqrEuclid<Point> tree = new SqrEuclid<Point>(2, null);
        
        // let's add some points to the kd-tree
        for(int i = 0; i < 10000; i++) {
            Point  p = new Point(rand.nextDouble() * 100, rand.nextDouble() * 100);
            tree.addPoint(p.xy, p);
        }
        
        // now we query the tree, asking what is the closest neighbor to a given
        // location
        // the returned value is a list of the "k" nearest neighbors (k is the
        // 2nd argument to the nearestNeighbor method). each "KdTree.Entry<T>"
        // object has the location and the associated data of the closest
        // neighbor.
        // the last argument indicates whether we want it to return a sorted list
        // but for some reason it seems to return points in the inverse order (!),
        // i.e. the point farther away comes first
        double[] queryPoint = new double[] { 5, 5 };
        int k  = 2;
        List<Entry<Point>> neighbors = tree.nearestNeighbor(queryPoint, 3, true);
        
        System.out.printf("%d nearest neighbors to (%.2f, %.2f)\n", k, queryPoint[0], queryPoint[1]);
        Collections.reverse(neighbors);
        for(Entry<Point> entry : neighbors) {
            System.out.printf("Location:(%.2f, %.2f); Distance: %.2f\n",
                    entry.value.xy[0], entry.value.xy[1], entry.distance);
        }
        
    }
}
