package april.vis;

import java.awt.*;
import java.io.*;
import javax.swing.*;

import april.jmat.*;

public class VisGrid implements VisObject, VisSerializable
{
    static int sz = 100; // number of lines to use when rendering grid.
    static float gridVertices[];
    static long gridId = VisUtil.allocateID();

    /** If non-null, specifies fixed spacing for gridlines in the x
     * and y directions.
     **/
    public double spacing[];

    /** The last spacing actually rendered. might be null. **/
    public double lastspacing[];

    Color gridColor, groundColor;

    public VisGrid()
    {
        this(new Color(128,128,128), new Color(32,32,32));
    }

    public VisGrid(Color gridColor, Color groundColor)
    {
        this.gridColor = gridColor;
        this.groundColor = groundColor;
    }

    static {
        // build (2*sz+1) lines that form a grid covering [-sz,-sz] to
        // [sz, sz]. Two vertices (four points) per line.
        gridVertices = new float[4*(2*sz+1)];

        int pos = 0;
        for (int i = -sz; i <= sz; i++) {
            float r = (float) Math.sqrt(sz*sz - i*i);
            gridVertices[pos++] = -r;
            gridVertices[pos++] = i;

            gridVertices[pos++] = r;
            gridVertices[pos++] = i;
        }
    }

    public synchronized void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        VisCameraManager.CameraPosition cameraPosition = rinfo.cameraPositions.get(layer);
        double M[][] = cameraPosition.getModelViewMatrix();

        double eye_dist = LinAlg.distance(cameraPosition.eye, cameraPosition.lookat);
        double spacingDefault = round_to_125(eye_dist / 10 );

        double spacingx = spacingDefault, spacingy = spacingDefault;
        if (spacing != null) {
            spacingx = spacing[0];
            spacingy = spacing[1];
        }

        lastspacing = new double[] { spacingx, spacingy };

        gl.glPushMatrix();

        double tx = Math.ceil(cameraPosition.lookat[0] / spacingx)*spacingx;
        double ty = Math.ceil(cameraPosition.lookat[1] / spacingy)*spacingy;

        gl.glTranslated(tx, ty, 0);

        gl.glMultMatrix(LinAlg.scale(spacingx, spacingy, 1));

        if (groundColor != null) {
            BasicShapes s = BasicShapes.circleFill16;

            gl.glEnable(GL.GL_POLYGON_OFFSET_FILL);
            gl.glPolygonOffset(5,5);

            gl.glLineWidth(1.0f);
            gl.glPushMatrix();
            gl.glMultMatrix(LinAlg.scale(sz, sz, sz));
            gl.glColor(groundColor);
            gl.gldBind(GL.VBO_TYPE_VERTEX, s.vid, s.vsz, s.vdim, s.v);
            gl.glDrawArrays(s.mode, 0, s.vsz);
            gl.gldUnbind(GL.VBO_TYPE_VERTEX, s.vid);
            gl.glPopMatrix();

            gl.glDisable(GL.GL_POLYGON_OFFSET_FILL);
        }

        if (gridColor != null) {
//            gl.glTranslated(0, 0, 0.01);
//            gl.glPolygonOffset(5,5);
            gl.glColor(gridColor);
            gl.gldBind(GL.VBO_TYPE_VERTEX, gridId, gridVertices.length/2, 2, gridVertices);

            gl.glDrawArrays(GL.GL_LINES, 0, gridVertices.length/2);
            gl.glMultMatrix(LinAlg.rotateZ(Math.PI/2));
            gl.glDrawArrays(GL.GL_LINES, 0, gridVertices.length/2);

            gl.gldUnbind(GL.VBO_TYPE_VERTEX, gridId);
        }

        gl.glPopMatrix();
    }

    /** round the input number to the next number of the form 1*10^n,
     * 2*10^n, or 5*10^n. */
    static double round_to_125(double in)
    {
        double v = 0.1; // minimum allowable value. Must be of form 1*10^n.

        while (v < in) {
            if (v < in)
                v *= 2;
            if (v < in)
                v = v/2 * 5;
            if (v < in)
                v *= 2;
        }

        return v;
    }

    /** Create a VisGrid with the default properties and a text overlay. **/
    public static VisGrid addGrid(VisWorld vw)
    {
        VisGrid vg = new VisGrid();

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("grid");
            vb.addFront(vg);
        }

        if (true) {
            VisWorld.Buffer vb = vw.getBuffer("grid-overlay");
            vb.setDrawOrder(10000);

            vb.addFront(new VisPixelCoordinates(VisPixelCoordinates.ORIGIN.CENTER_ROUND,
                                                new VisDepthTest(false,
                                                                 // translate so we don't draw on top of canvas dimensions.
                                                                 LinAlg.translate(0, -30, 0),
                                                                 new VisGridText(vg, VisText.ANCHOR.CENTER_ROUND,
                                                                                 "<<sansserif-12>>grid %.2f m"))));
        }

        return vg;
    }

    public VisGrid(ObjectReader r)
    {
    }

    public void writeObject(ObjectWriter outs) throws IOException
    {
        outs.writeDoubles(spacing);
        outs.writeColor(gridColor);
        outs.writeColor(groundColor);
    }

    public void readObject(ObjectReader ins) throws IOException
    {
        spacing = ins.readDoubles();
        gridColor = ins.readColor();
        groundColor = ins.readColor();
    }
}
