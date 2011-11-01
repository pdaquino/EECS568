package april.vis;

import java.util.*;
import java.awt.*;

public class VisPoints implements VisObject
{
    VisAbstractVertexData vd;
    VisAbstractColorData cd;
    double pointSize;

    public VisPoints(VisAbstractVertexData vd, VisAbstractColorData cd, double pointSize)
    {
        this.vd = vd;
        this.cd = cd;
        this.pointSize = pointSize;
    }

    public synchronized void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        vd.bind(gl);
        cd.bind(gl);

        gl.glNormal3f(0, 0, 1);

        gl.glPointSize((float) pointSize);
        gl.glDrawArrays(GL.GL_POINTS, 0, vd.size());

        cd.unbind(gl);
        vd.unbind(gl);
    }
}
