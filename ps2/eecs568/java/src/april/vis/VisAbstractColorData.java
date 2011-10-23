package april.vis;

public interface VisAbstractColorData
{
    public void bind(GL gl);
    public void unbind(GL gl);

    /** returns the number of colors, or -1 if any number of vertices
        can be colored. **/
    public int size();
}
