package april.vis;

import java.util.*;
import java.awt.*;

public class VisConstantColor implements VisAbstractColorData
{
    Color c;

    public VisConstantColor(Color c)
    {
        this.c = c;
    }

    public synchronized int size()
    {
        return -1;
    }

    public synchronized void bind(GL gl)
    {
        gl.glColor(c);
    }

    public synchronized void unbind(GL gl)
    {
        // nop
    }

}
