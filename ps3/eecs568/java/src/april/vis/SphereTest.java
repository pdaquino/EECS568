package april.vis;

import java.awt.*;
import java.awt.image.*;
import javax.swing.*;
import java.util.*;
import april.jmat.*;
import april.util.*;

import java.io.*;
import javax.imageio.*;

public class SphereTest implements ParameterListener
{
    JFrame jf;
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    VisTexture tex;

    ParameterGUI pg = new ParameterGUI();

    public SphereTest()
    {
        pg.addIntSlider("iters", "iterations", 0, 10, 4);

        jf = new JFrame("SphereTest");
        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);

        try {
//            BufferedImage im = ImageIO.read(new File("1_earth_8k.jpg"));
            BufferedImage im = ImageIO.read(new File("earth.png"));
            tex = new VisTexture(im, false);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }

        jf.setSize(600,400);
        jf.setVisible(true);

        pg.addListener(this);
        parameterChanged(pg, "iters");
    }

    static double sq(double v)
    {
        return v*v;
    }

    static class SphereGeometry
    {
        ArrayList<float[]> vertices = new ArrayList<float[]>();
        ArrayList<float[]> texcoords = new ArrayList<float[]>();
        ArrayList<int[]> tris = new ArrayList<int[]>();

        float  _verts[];
        float  _texcoords[];
        int    _indices[];

        HashMap<Float,IntArray> vertexHash = new HashMap<Float,IntArray>();

        // return the index of a vertex having position v and texcoord
        // s, recycling an existing vertex if possible, else creating
        // a new vertex.
        int addVertexTexCoord(float v[], float s[])
        {
            IntArray ia = vertexHash.get(10*s[0] + s[1]);

            if (ia != null) {

                // check all indices in ia.
                for (int j = 0; j < ia.size(); j++) {
                    int i = ia.get(j);

                    float w[] = vertices.get(i);

                    if (v[0]==w[0] && v[1]==w[1] && v[2]==w[2]) {

                        float t[] = texcoords.get(i);

                        if (s[0]==t[0] && s[1]==t[1]) {
                            return i;
                        }
                    }
                }
            } else {
                ia = new IntArray();
                vertexHash.put(10*s[0] + s[1], ia);
            }

            int idx = vertices.size();
            vertices.add(v);
            texcoords.add(s);

            ia.add(idx);

            return idx;
        }

        // Returns an array containing the indices of vertices that belong to the triangle.
        int[] makeTriangle(float va[], float vb[], float vc[])
        {
            // compute texture coordinates
            float sa[] = new float[] { (float) (Math.atan2(va[1], va[0])/(2*Math.PI)),
                                       (float) (Math.acos(va[2])/(Math.PI)) };

            float sb[] = new float[] { (float) mod1(sa[0], Math.atan2(vb[1], vb[0])/(2*Math.PI)),
                                       (float) (Math.acos(vb[2])/(Math.PI)) };

            float sc[] = new float[] { (float) mod1(sa[0], Math.atan2(vc[1], vc[0])/(2*Math.PI)),
                                       (float) (Math.acos(vc[2])/(Math.PI))};

            int a = addVertexTexCoord(va, sa);
            int b = addVertexTexCoord(vb, sb);
            int c = addVertexTexCoord(vc, sc);

            return new int[] { a, b, c };
        }

        SphereGeometry(int refinesteps)
        {
            if (true) {
                float v = (float) (Math.sqrt(3)/3);

                float va[] = new float[] {  v,  v,  v };
                float vb[] = new float[] { -v, -v,  v };
                float vc[] = new float[] { -v,  v, -v };
                float vd[] = new float[] {  v, -v, -v };

                tris.add(makeTriangle(va, vb, vc));
                tris.add(makeTriangle(va, vd, vb));
                tris.add(makeTriangle(va, vc, vd));
                tris.add(makeTriangle(vb, vd, vc));
            }

            // refine it
            for (int refine = 0; refine < refinesteps; refine++) {

                ArrayList<int[]> newtris = new ArrayList<int[]>();

                // sub-divide every triangle into four new triangles.
                for (int tri[] : tris) {
                    int a = tri[0], b = tri[1], c = tri[2];

                    int ab = vertices.size(), bc = vertices.size()+1, ac = vertices.size() + 2;

                    float va[] = vertices.get(a);
                    float vb[] = vertices.get(b);
                    float vc[] = vertices.get(c);

                    float vab[] = LinAlg.normalize(new float[] { va[0]+vb[0],
                                                                 va[1]+vb[1],
                                                                 va[2]+vb[2] });

                    float vbc[] = LinAlg.normalize(new float[] { vc[0]+vb[0],
                                                                 vc[1]+vb[1],
                                                                 vc[2]+vb[2] });

                    float vac[] = LinAlg.normalize(new float[] { vc[0]+va[0],
                                                                 vc[1]+va[1],
                                                                 vc[2]+va[2] });

                    newtris.add(makeTriangle(va, vab, vac));
                    newtris.add(makeTriangle(vab, vb, vbc));
                    newtris.add(makeTriangle(vac, vab, vbc));
                    newtris.add(makeTriangle(vc, vac, vbc));
                }

                tris = newtris;
            }

            // create compact geometry structures.
            _verts = new float[vertices.size()*3];
            _texcoords = new float[texcoords.size()*2];
            _indices = new int[tris.size()*3];

            for (int i = 0; i < vertices.size(); i++) {
                float v[] = vertices.get(i);
                _verts[3*i+0] = v[0];
                _verts[3*i+1] = v[1];
                _verts[3*i+2] = v[2];

                float s[] = texcoords.get(i);
                _texcoords[2*i+0] = s[0];
                _texcoords[2*i+1] = s[1];
            }

            for (int i = 0; i < tris.size(); i++) {
                int tri[] = tris.get(i);
                _indices[3*i+0] = tri[0];
                _indices[3*i+1] = tri[1];
                _indices[3*i+2] = tri[2];
            }
        }
    }

    public synchronized void parameterChanged(ParameterGUI pg, String name)
    {
        SphereGeometry geom = new SphereGeometry(pg.gi("iters"));

        System.out.printf("%d vertices, %d triangles\n", geom.vertices.size(), geom.tris.size());

        VisWorld.Buffer vb = vw.getBuffer("sphere");
        vb.addBack(new MyObject(tex, geom));
        vb.swap();
    }

    static final double mod1(double ref, double v)
    {
        while (v - ref > .5)
            v -= 1;

        while (v - ref < -.5)
            v += 1;

        return v;
    }


    class MyObject implements VisObject
    {
        long vid, tid, iid;

        float verts[], texcoords[];
        int indices[];

        VisTexture tex;

        public MyObject(VisTexture tex, SphereGeometry geom)
        {
            this.tex = tex;
            this.verts = geom._verts;
            this.texcoords = geom._texcoords;
            this.indices = geom._indices;

            vid = VisUtil.allocateID();
            tid = VisUtil.allocateID();
            iid = VisUtil.allocateID();
        }

        public void render(VisCanvas _vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
        {
            tex.bind(gl);

            gl.glScaled(20,20,20);

            gl.glColor(Color.white);

            gl.gldBind(GL.VBO_TYPE_VERTEX, vid, verts.length / 3, 3, verts);
            gl.gldBind(GL.VBO_TYPE_NORMAL, vid, verts.length / 3, 3, verts);
            gl.gldBind(GL.VBO_TYPE_TEX_COORD, tid, texcoords.length / 2, 2, texcoords);
            gl.gldBind(GL.VBO_TYPE_ELEMENT_ARRAY, iid, indices.length, 1, indices);

            gl.glDrawRangeElements(GL.GL_TRIANGLES, 0, verts.length / 3, indices.length, 0);

            gl.gldUnbind(GL.VBO_TYPE_VERTEX, vid);
            gl.gldUnbind(GL.VBO_TYPE_NORMAL, vid);
            gl.gldUnbind(GL.VBO_TYPE_TEX_COORD, tid);
            tex.unbind(gl);
        }
    }

    public static void main(String args[])
    {
        new SphereTest();
    }
}
