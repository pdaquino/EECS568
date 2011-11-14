package team;

import java.util.*;

import april.jmat.*;

class BinPoints
{
    HashSet<BinKey> hits = new HashSet<BinKey>();
    double binsize;

    public BinPoints(ArrayList<double[]> points, double binsize)
    //public BinPoints(ArrayList<Task2.Line> lines, double binsize)
    {
        for (double[] p: points) {
            BinKey bk = new BinKey(p, binsize);
            hits.add(bk);
        }
        /*for (Task2.Line l: lines) {
            ArrayList<double[]> ls = l.getLineSeg();
            double[] n = LinAlg.subtract(ls.get(1), ls.get(0));
            double stepsize = binsize*.5;
            int iters = (int)Math.ceil(LinAlg.magnitude(n)/stepsize);
            n = LinAlg.normalize(n);
            double[] p = LinAlg.copy(ls.get(0));
            for (int i = 0; i < iters; i++) {
                BinKey bk = new BinKey(p, binsize);
                hits.add(bk);

                p[0] += n[0]*stepsize;
                p[1] += n[1]*stepsize;
            }

        }*/
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
