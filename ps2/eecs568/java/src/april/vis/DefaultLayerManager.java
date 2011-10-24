package april.vis;

import java.util.*;

import april.jmat.*;

/** Layer positions are given in Java window coordinates, i.e., with
 * 0,0 in the upper left.
 **/
public class DefaultLayerManager implements VisLayerManager
{
    HashMap<VisLayer, LayerPosition> layerPositions = new HashMap<VisLayer, LayerPosition>();

    // default constructor
    public DefaultLayerManager()
    {
    }

    public DefaultLayerManager(VisLayer vl, double pos[])
    {
        LayerPosition lpos = new LayerPosition();
        lpos.dpos1 = LinAlg.copy(pos);

        layerPositions.put(vl, lpos);
    }

    static class LayerPosition
    {
        // The position of a layer is determined by four numbers:
        //
        // 0: the left most coordinate as a fraction of the viewport width
        // 1: the top most coordinate as a fraction of the viewport height
        // 2: the width as a fraction of the viewport width
        // 3: the height as a fraction of the viewport height

        double dpos0[];
        long mtime0;
        double dpos1[] = new double[] { 0, 0, 1, 1};
        long mtime1 = System.currentTimeMillis();

        double[] getPosition(long mtime)
        {
            if (dpos0 == null || mtime >= mtime1)
                return dpos1;

            if (mtime <= mtime0 && dpos0 != null)
                return dpos0;

            double alpha1 = ((double) mtime - mtime0) / (mtime1 - mtime0);
            double alpha0 = 1.0 - alpha1;

            return new double[] { alpha0 * dpos0[0] + alpha1 * dpos1[0],
                                  alpha0 * dpos0[1] + alpha1 * dpos1[1],
                                  alpha0 * dpos0[2] + alpha1 * dpos1[2],
                                  alpha0 * dpos0[3] + alpha1 * dpos1[3] };
        }
    }

    public int[] getLayerPosition(VisCanvas vc, int viewport[], VisLayer vl, long mtime)
    {
        LayerPosition layerPosition = layerPositions.get(vl);
        if (layerPosition == null) {
            layerPosition = new LayerPosition();
            layerPositions.put(vl, layerPosition);
            // XXX memory leak if many layers are created and discarded.
        }

        double dpos[] = layerPosition.getPosition(mtime);

        return new int[] { (int) Math.round(viewport[2]*dpos[0]),
                           (int) Math.round(viewport[3]*dpos[1]),
                           (int) Math.round(viewport[2]*dpos[2]),
                           (int) Math.round(viewport[3]*dpos[3]) };
    }
}
