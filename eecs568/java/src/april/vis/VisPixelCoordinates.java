package april.vis;

/** Changes the model view and projection matrix to create a one unit
 * to one pixel projection. The origin of this projection can be
 * configured to be various parts of the screen, which are well-suited
 * to positioning text or images as overlayed data. The resulting
 * coordinate system will be an OpenGL-style coordinate system, where
 * y=0 is on the bottom (not the top). **/
public class VisPixelCoordinates implements VisObject
{
    public enum ORIGIN { TOP_LEFT, TOP, TOP_RIGHT, LEFT, CENTER, RIGHT, BOTTOM_LEFT, BOTTOM, BOTTOM_RIGHT };

    ORIGIN origin;
    VisObject vo;

    public VisPixelCoordinates(ORIGIN origin, VisObject vo)
    {
        this.origin = origin;
        this.vo = vo;
    }

    public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        gl.glMatrixMode(GL.GL_PROJECTION);
        gl.glPushMatrix();

        gl.glMatrixMode(GL.GL_MODELVIEW);
        gl.glPushMatrix();

        gl.glMatrixMode(GL.GL_PROJECTION);
        gl.glLoadIdentity();

        gl.glMatrixMode(GL.GL_MODELVIEW);

        int viewport[] = rinfo.layerPositions.get(layer);

        double width = viewport[2];
        double height = viewport[3];

        switch (origin) {
            case BOTTOM_LEFT:
                // nothing to do.
                break;

            case LEFT:
                gl.glTranslated(0, height/2, 0);
                break;

            case TOP_LEFT:
                gl.glTranslated(0, height, 0);
                break;
            case TOP:
                gl.glTranslated(width/2, height, 0);
                break;
            case TOP_RIGHT:
                gl.glTranslated(width, height, 0);
                break;

            default:
                System.out.println("Unsupported Origin");
                break;
        }

        vo.render(vc, layer, rinfo, gl);
        gl.glMatrixMode(GL.GL_PROJECTION);
        gl.glPopMatrix();

        gl.glMatrixMode(GL.GL_MODELVIEW);
        gl.glPopMatrix();

    }

}
