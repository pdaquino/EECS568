package april.vis;

import java.awt.event.*;
import april.jmat.geom.*;

public interface VisEventHandler
{
    /** Higher priorities get first dibs on events. **/
    public int getPriority();

    /** Return true if you've consumed the event. **/
    public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e);
    public boolean mouseReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e);
    public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e);
    public boolean mouseDragged(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e);
    public boolean mouseMoved(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e);
    public boolean mouseWheel(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseWheelEvent e);

    public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e);
    public boolean keyTyped(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e);
    public boolean keyReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e);
}
