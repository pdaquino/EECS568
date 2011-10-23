package april.vis;

import java.util.*;
import java.awt.*;

public class VisLines implements VisObject
{
    VisAbstractVertexData vd;
    VisAbstractColorData cd;
    double lineWidth;

    public enum TYPE { LINES, LINE_LOOP, LINE_STRIP };

    TYPE type;

    public VisLines(VisAbstractVertexData vd, VisAbstractColorData cd, double lineWidth, TYPE type)
    {
        this.vd = vd;
        this.cd = cd;
        this.lineWidth = lineWidth;
        this.type = type;
    }

    public synchronized void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        vd.bind(gl);
        cd.bind(gl);

        gl.glNormal3f(0, 0, 1);
        gl.glLineWidth((float) lineWidth);

        if (type == TYPE.LINES)
            gl.glDrawArrays(GL.GL_LINES, 0, vd.size());
        else if (type == TYPE.LINE_STRIP)
            gl.glDrawArrays(GL.GL_LINE_STRIP, 0, vd.size());
        else if (type == TYPE.LINE_LOOP)
            gl.glDrawArrays(GL.GL_LINE_LOOP, 0, vd.size());

        cd.unbind(gl);
        vd.unbind(gl);
    }
}
