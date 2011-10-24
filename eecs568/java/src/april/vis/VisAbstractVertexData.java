package april.vis;

public interface VisAbstractVertexData
{
    public void bind(GL gl);
    public void unbind(GL gl);

    /** returns the number of vertices. **/
    public int size();
}
