package team;


import java.awt.*;
import java.util.*;

import april.vis.*;

//placeholder class for when polygons are working again
public class VisRobot implements VisObject
{

    VisLines vlines;

    public VisRobot(Color c)
    {
        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        rpoints.add(new double[]{-.3, .3});
        rpoints.add(new double[]{-.3, -.3});
        rpoints.add(new double[]{.45,0});
        VisVertexData robotData = new VisVertexData(rpoints);
        vlines = new VisLines(robotData, new VisConstantColor(c), 2, VisLines.TYPE.LINE_LOOP);
    }

    public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        vlines.render(vc,layer,rinfo,gl);
    }

}