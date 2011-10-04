package april.vis;

import java.awt.*;
import java.awt.image.*;
import javax.swing.*;
import java.util.*;
import april.jmat.*;

public class VisTest
{
    JFrame jf;
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    VisFont vfont = new VisFont(new Font("Sans Serif", Font.PLAIN, 128));

    static class MyLayer
    {
        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);

        MyLayer(double pos[], Color c)
        {
            vl.backgroundColor = c;
            vl.layerManager = new DefaultLayerManager(vl, pos);

            VisWorld.Buffer vb = vw.getBuffer("foo");
            vb.addBack(new VisChain(LinAlg.scale(10, 10, 10),
                                        new Square(Color.blue),
                                        LinAlg.translate(-0.1, -0.1, .1),
                                        new Square(Color.green)
                               ));
            vb.swap();
        }
    }

    static class PlotLayer
    {
        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);

        PlotLayer(double pos[])
        {
            vl.layerManager = new DefaultLayerManager(vl, pos);
//            ((DefaultCameraManager) vl.cameraManager).perspective_fovy_degrees = 30;
//            ((DefaultCameraManager) vl.cameraManager).perspectiveness = 0;
            ((DefaultCameraManager) vl.cameraManager).scaley = 10;
            ((DefaultCameraManager) vl.cameraManager).interfaceMode = 2.5;
            ((DefaultCameraManager) vl.cameraManager).fit2D(new double[] { 0, -1, 0},
                                                            new double[] { 30, 1, 0 },
                                                            true);
            vl.backgroundColor = Color.red;


            VisGrid vg = new VisGrid(new Color(50,50,50), new Color(70,70,70));
            vg.spacing = new double[] { Math.PI/2, 1 };

            VisWorld.Buffer vb = vw.getBuffer("plot");
            vb.setDrawOrder(-100);
            vb.addBack(new VisDepthTest(false, vg));

            VisColorData cd = new VisColorData();
            VisVertexData vd = new VisVertexData();
            for (double x = 0; x < 30; x += 0.01) {
                float p[] = new float[] { (float) x, (float) Math.sin(x) };
                vd.add(p);
                cd.add(0x1f0000 + (int) (255*(.5+p[1]/2)));
            }
            vb.addBack(new VisLighting(false, new VisPoints(vd, cd, 2.0)));
            vb.swap();
        }
    }

    public VisTest()
    {
        jf = new JFrame("VisTest");
        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.setSize(600,400);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("grid");
            vb.addBack(new VisDepthTest(false, new VisGrid()));
            vb.swap();
        }

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("foo");
            vb.addBack(new VisChain(LinAlg.scale(10, 10, 10),
                                        new Square(Color.blue),
                                        LinAlg.translate(-0.1, -0.1, 1),
                                        new Square(Color.green)
                               ));

            vb.swap();
        }

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("visfont");
            vb.addBack(new VisChain(LinAlg.translate(0,0,1),
                                        LinAlg.scale(0.1, 0.1, 0.1),
                                        vfont.makeText("blueberry sunday", Color.cyan)));

            vb.swap();
        }

        if (true) {
            int width = 100;
            int height = 50;
            BufferedImage im = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);

            if (false) {
                Graphics2D g = im.createGraphics();
                g.setColor(Color.black);
                g.fillRect(0,0, width, height);
                g.setColor(Color.yellow);
                g.fillOval(0,0,width,height);
                g.dispose();
            } else {
                for (int y = 0; y < height; y++)
                    for (int x = 0; x < width; x++)
                        im.setRGB(x, y, ((x&1)^(y&1))==0 ? 0xff000000 : 0xffffffff);
            }

//            VisTexture vt = new VisTexture(im, true);
            VisWorld.Buffer vb = vw.getBuffer("image");
            vb.setDrawOrder(100);
            vb.addBack(new VisChain(LinAlg.translate(15, 0, 0.5),
                                        new VisImage(vfont.texture,
                                                     new double[][] { { 0, 0, 0 },
                                                                      { 10, 0, 0 },
                                                                      { 10, 10, 0 },
                                                                      { 0, 10, 0 } },
                                                     new double[][] { { 0, 0 },
                                                                      { 1, 0 },
                                                                      { 1, 1 },
                                                                      { 0, 1 } },
                                                     new Color(255,0,0,255))));
            vb.swap();
        }

        if (true) {
            MyLayer ml1 = new MyLayer(new double[] { 0, 0, 0.25, 0.25 }, Color.blue);
            vc.addLayer(ml1.vl);

            MyLayer ml2 = new MyLayer(new double[] { 0.75, 0.75, 0.25, 0.25 }, Color.green);
            vc.addLayer(ml2.vl);

            PlotLayer pl = new PlotLayer(new double[] { .0, 0.6, 0.4, 0.4 });
            vc.addLayer(pl.vl);
        }
    }

    public static void main(String args[])
    {
        VisTest vt = new VisTest();
    }

    class DumbSquare implements VisObject
    {
        Color c;

        public DumbSquare(Color c)
        {
            this.c = c;
        }

        public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
        {
            gl.glColor(c);

            gl.glBegin(GL.GL_QUADS);
            gl.glVertex2d(-.5, -.5);
            gl.glVertex2d(.5, -.5);
            gl.glVertex2d(.5, .5);
            gl.glVertex2d(-.5, .5);
            gl.glEnd();
        }
    }

    static class Square implements VisObject
    {
        static long id,id2;
        static float vertices[];
        static int colors[];

        static {
            id = VisUtil.allocateID();
            id2 = VisUtil.allocateID();

            vertices = new float[] { -.5f, -.5f,
                                     .5f, -.5f,
                                     .5f, .5f,
                                     -.5f, .5f };

            // AARRGGBB
            colors = new int[] { 0xffff0000,
                                 0xff00ff00,
                                 0xff0000ff,
                                 0xffffffff };
        }

        Color c;

        public Square(Color c)
        {
            this.c = c;
        }

        public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
        {
            gl.glColor(c);
            gl.gldBind(GL.VBO_TYPE_VERTEX, id, 4, 2, vertices);
            gl.gldBind(GL.VBO_TYPE_COLOR, id2, 4, 4, colors);
            gl.glDrawArrays(GL.GL_QUADS, 0, 4);
            gl.gldUnbind(GL.VBO_TYPE_VERTEX, id);
            gl.gldUnbind(GL.VBO_TYPE_COLOR, id2);
        }
    }
}
