package rgbdslam;

import java.util.*;
import april.jmat.*;

class BinPoints
{
    HashSet<BinKey> hits = new HashSet<BinKey>();
    double binsize;

    public BinPoints(ArrayList<double[]> points, double binsize)
    {
        for (double[] p: points) {
            BinKey bk = new BinKey(p, binsize);
            hits.add(bk);
        }
        this.binsize = binsize;
    }

    public boolean hit(double[] xy)
    {
        BinKey bk = new BinKey(xy, binsize);
        return hits.contains(bk);
    }

    class BinKey
    {
        int x,y;

        public BinKey(double[] xy, double binsize)
        {
            x = (int)(xy[0]/binsize);
            y = (int)(xy[1]/binsize);
        }

        public int hashCode()
        {
            Integer X = new Integer(x);
            Integer Y = new Integer(y);
            return X.hashCode()^Y.hashCode();
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof BinKey))
                return false;
            BinKey b = (BinKey)o;
            return b.x == x && b.y == y;
        }
    }
}
