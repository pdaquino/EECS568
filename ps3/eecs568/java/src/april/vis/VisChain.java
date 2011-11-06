package april.vis;

import java.util.*;

public class VisChain implements VisObject
{
    ArrayList<Object> ops = new ArrayList<Object>();

    public VisChain()
    {
    }

    public VisChain(Object ... os)
    {
        add(os);
    }

    // this method must be added to disabiguate between a
    // two-dimensional array being interpreted as a varargs call
    // consisting of several one-dimensional doubles.
    public void add(double M[][])
    {
        ops.add(M);
    }

    public void add(Object ... os)
    {
        int i = 0;

        while (i < os.length) {
            if (os[i] == null) {
                i++;
                continue;
            }

            if (os[i] instanceof double[][]) {
                ops.add(os[i]);
                i++;
                continue;
            }

            if (os[i] instanceof VisObject) {
                ops.add((VisObject) os[i]);
                i++;
                continue;
            }

            // unknown type!
            System.out.println("VisChain: Unknown object added to chain: "+os[i]);
            assert(false);
            i++;
        }
    }

    public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        gl.glPushMatrix();

        for (Object o : ops) {

            if (o instanceof double[][]) {
                gl.glMultMatrix((double[][]) o);
                continue;
            }

            if (o instanceof VisObject) {
                VisObject vo = (VisObject) o;
                vo.render(vc, layer, rinfo, gl);
            }
        }

        gl.glPopMatrix();
    }


}
