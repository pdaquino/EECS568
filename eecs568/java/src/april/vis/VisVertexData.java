package april.vis;

import java.util.*;
import java.awt.*;

public class VisVertexData implements VisAbstractVertexData
{
    long id = -1; // allocate an id lazily
    ArrayList<Block> blocks = new ArrayList<Block>();

    static class Block
    {
        float vf[];
        double vd[];
        int nv;
        int dim;
    }

    public VisVertexData()
    {
    }

    public VisVertexData(ArrayList<double[]> d)
    {
        add(d);
    }

    /** Add a single vertex: PERFORMANCE WARNING. **/
    public synchronized void add(float v[])
    {
        add(v, 1, v.length);
    }

    /** Add a single vertex: PERFORMANCE WARNING. **/
    public synchronized void add(double v[])
    {
        add(v, 1, v.length);
    }

    /** Add multiple vertices **/
    public synchronized void add(float v[], int nv, int dim)
    {
        Block b = new Block();
        b.vf = v;
        b.nv = nv;
        b.dim = dim;

        blocks.add(b);
    }

    /** Add multiple vertices **/
    public synchronized void add(double v[], int nv, int dim)
    {
        Block b = new Block();
        b.vd = v;
        b.nv = nv;
        b.dim = dim;

        blocks.add(b);
    }

    /** Add multiple vertices **/
    public synchronized void add(ArrayList<double[]> points)
    {
        int dim = 2;
        for (double p[] : points) {
            if (p.length >= 3)
                dim = 3;
        }

        Block b = new Block();
        b.vd = new double[points.size()*dim];

        for (int i = 0; i < points.size(); i++) {
            double p[] = points.get(i);

            b.vd[i*dim+0] = p[0];
            b.vd[i*dim+1] = p[1];
            if (p.length > 2)
                b.vd[i*dim+2] = p[2];
        }

        b.nv = points.size();
        b.dim = dim;

        blocks.add(b);
    }

    public synchronized int size()
    {
        combine();

        if (blocks.size() == 0)
            return 0;

        return blocks.get(0).nv;
    }

    synchronized void combine()
    {
        if (blocks.size() <= 1)
            return;

        boolean doubles = false;
        int dim = 2;
        int nv = 0;

        for (Block b : blocks) {
            if (b.vd != null)
                doubles = true;
            if (b.dim == 3)
                dim = 3;

            nv += b.nv;
        }

        if (doubles) {
            // must promote all to doubles

            double vd[] = new double[dim*nv];
            int pos = 0;

            for (Block b : blocks) {
                if (b.vf != null) {
                    for (int i = 0; i < b.nv; i++) {
                        for (int j = 0; j < b.dim; j++) {
                            vd[pos*dim+j] = b.vf[i*b.dim+j];
                        }
                        pos++;
                    }
                } else {
                    for (int i = 0; i < b.nv; i++) {
                        for (int j = 0; j < b.dim; j++) {
                            vd[pos*dim+j] = b.vd[i*b.dim+j];
                        }
                        pos++;
                    }
                }
            }

            Block b = new Block();
            b.vd = vd;
            b.nv = nv;
            b.dim = dim;
            blocks.clear();
            blocks.add(b);

        } else {
            // keep as floats
            float vf[] = new float[dim*nv];
            int pos = 0;

            for (Block b : blocks) {
                for (int i = 0; i < b.nv; i++) {
                    for (int j = 0; j < b.dim; j++) {
                        vf[pos*dim+j] = b.vf[i*b.dim+j];
                    }
                    pos++;
                }
            }

            Block b = new Block();
            b.vf = vf;
            b.nv = nv;
            b.dim = dim;
            blocks.clear();
            blocks.add(b);
        }

        id = -1;
    }

    public synchronized void bind(GL gl)
    {
        combine();

        if (id < 0)
            id = VisUtil.allocateID();

        Block b = blocks.get(0);

        if (b.vf != null)
            gl.gldBind(GL.VBO_TYPE_VERTEX, id, b.nv, b.dim, b.vf);
        else
            gl.gldBind(GL.VBO_TYPE_VERTEX, id, b.nv, b.dim, b.vd);
    }

    public synchronized void unbind(GL gl)
    {
        gl.gldUnbind(GL.VBO_TYPE_VERTEX, id);
    }

    public synchronized void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        if (blocks.size() == 0)
            return;

        if (blocks.size() > 1)
            combine();

        if (id < 0)
            id = VisUtil.allocateID();

        Block b = blocks.get(0);
        if (b.vf != null)
            gl.gldBind(GL.VBO_TYPE_VERTEX, id, b.nv, b.dim, b.vf);
        else
            gl.gldBind(GL.VBO_TYPE_VERTEX, id, b.nv, b.dim, b.vd);

        gl.glColor(Color.blue);
        gl.glPointSize(2.0f);
        gl.glDrawArrays(GL.GL_POINTS, 0, b.nv);
    }
}
