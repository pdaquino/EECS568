package april.vis;

public class VisLighting extends VisChain
{
    boolean enabled;

    public VisLighting(boolean enabled, Object ... os)
    {
        super (os);

        this.enabled = enabled;
    }

    public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        gl.glPushAttrib(gl.GL_ENABLE_BIT);

        if (!enabled)
            gl.glDisable(GL.GL_LIGHTING);
        else
            gl.glEnable(GL.GL_LIGHTING);

        super.render(vc, layer, rinfo, gl);

        gl.glPopAttrib();
    }
}
