package april.vis;

import april.jmat.geom.*;

/** Picks a manipulation point based on the intersection of the ray
 * with the XY plane (by default at z=0) **/
public class DefaultManipulationManager implements VisManipulationManager
{
    public double z = 0;

    public double[] pickManipulationPoint(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray)
    {
        return ray.intersectPlaneXY(z);
    }
}
