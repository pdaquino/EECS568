package april.vis;

import java.awt.*;
import java.awt.image.*;

import april.jmat.*;

public class VisImage implements VisObject
{
    VisTexture texture;
    double vertices[][];
    double texcoords[][];
    Color c;

    public VisImage(VisTexture texture, double vertices[][], double texcoords[][], Color c)
    {
        this.texture = texture;
        this.vertices = LinAlg.copy(vertices);
        this.texcoords = LinAlg.copy(texcoords);
        this.c = c;
    }

    public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        gl.glColor(c);

        texture.bind(gl);

        gl.glBegin(gl.GL_QUADS);

        for (int i = 0; i < 4; i++) {
            gl.glTexCoord2d(texcoords[i][0], texcoords[i][1]);
            gl.glVertex3d(vertices[i][0], vertices[i][1], vertices[i][2]);
        }

        gl.glEnd();

        texture.unbind(gl);
    }
}
