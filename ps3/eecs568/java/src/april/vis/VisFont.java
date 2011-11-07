package april.vis;

import java.awt.*;
import java.awt.geom.*;
import java.awt.image.*;
import javax.imageio.*;
import java.io.*;

public class VisFont
{
    VisTexture texture;
    Font f;
    FontMetrics fm;

    static final int ASCII_MIN = ' ', ASCII_MAX = '~';

    int tile_width, tile_height; // size of each letter's tile (in pixels)
    int ntiles; // how many tiles in total?
    int tile_dim; // how many tiles in each row?

    int width, height; // overall size of our texture

    public VisFont(Font f)
    {
        this.f = f;
        BufferedImage tmp = new BufferedImage(1, 1, BufferedImage.TYPE_INT_ARGB);
        Graphics tmpg = tmp.createGraphics();
        fm = tmpg.getFontMetrics(f);

        tile_width = fm.getMaxAdvance();
        tile_height = fm.getMaxAscent() + fm.getMaxDescent();

        ntiles = ASCII_MAX - ASCII_MIN + 1;
        tile_dim = ((int) Math.sqrt(ntiles)) + 1; // how many rows and columns of tiles?

        width = tile_dim*tile_width;
        height = tile_dim*tile_height;

        width = width + (4 - (width&3));
        height = height + (4 - (height&3));

//        BufferedImage tiles = new BufferedImage(width, height, BufferedImage.TYPE_BYTE_GRAY);
        BufferedImage tiles = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g = tiles.createGraphics();

        g.setFont(f);
        g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g.setRenderingHint(RenderingHints.KEY_RENDERING, RenderingHints.VALUE_RENDER_QUALITY);

        g.setColor(new Color(255,255,255,0));
        g.fillRect(0, 0, width, height);
        g.setColor(new Color(255,255,255,255));

        for (int idx = 0; idx < ntiles; idx++) {
            String s = ""+((char) (idx+ASCII_MIN));

            int tile_y = idx / tile_dim;
            int tile_x = idx - tile_y * tile_dim;

            g.drawString(s, tile_x*tile_width, tile_y*tile_height + fm.getMaxAscent());
        }

        if (true) {
            int idata[] = ((DataBufferInt) (tiles.getRaster().getDataBuffer())).getData();
            for (int i = 0; i < idata.length; i++) {
//            System.out.printf("%08x\n", idata[i]);
                int alpha = (idata[i]>>24)&0xff;
                idata[i] = alpha | (alpha<<8) | (alpha<<16) | (alpha<<24);
/*                int limit = 255;
                if (alpha != 255 && alpha > limit)
                    alpha = limit;
                    idata[i] = (alpha <<24) | 0xffffff;*/
            }
        }

        if (false) {
            byte data[] = ((DataBufferByte) (tiles.getRaster().getDataBuffer())).getData();
            for (int i = 0; i < data.length; i++) {
                int v = data[i] & 0xff;
                int limit = 128;
                if (v != 255 && v > limit)
                    data[i] = (byte) limit;
            }
        }

        texture = new VisTexture(tiles, false);

        g.dispose();
        tmpg.dispose();
    }

    public VisObject makeText(String s, Color color)
    {
        return new Text(s, color);
    }

    class Text implements VisObject
    {
        float verts[];
        float texcoords[];

        long vertsID = VisUtil.allocateID();
        long texCoordsID = VisUtil.allocateID();

        Color color;

        public Text(String s, Color color)
        {
            this.color = color;

            verts = new float[s.length() * 8];
            texcoords = new float[s.length() * 8];

            float xpos = 0;


            for (int i = 0; i < s.length(); i++) {
                char c = s.charAt(i);
                if (c < ASCII_MIN || c > ASCII_MAX)
                    c = ' ';

                float char_width = fm.charWidth(c);

                int idx = c - ASCII_MIN;
                int tile_y = idx / tile_dim;
                int tile_x = idx - (tile_y*tile_dim);

                verts[8*i+0]     = xpos;
                verts[8*i+1]     = 0;
                texcoords[8*i+0] = tile_x*tile_width;
                texcoords[8*i+1] = (tile_y+1)*tile_height;

                verts[8*i+2]     = xpos+char_width;
                verts[8*i+3]     = 0;
                texcoords[8*i+2] = tile_x*tile_width + char_width;
                texcoords[8*i+3] = (tile_y+1)*tile_height;

                verts[8*i+4]     = xpos+char_width;
                verts[8*i+5]     = tile_height;
                texcoords[8*i+4] = tile_x*tile_width + char_width;
                texcoords[8*i+5] = (tile_y+0)*tile_height;

                verts[8*i+6]     = xpos;
                verts[8*i+7]     = tile_height;
                texcoords[8*i+6] = tile_x*tile_width;
                texcoords[8*i+7] = (tile_y+0)*tile_height;

                xpos += char_width;
            }

            for (int i = 0; i < texcoords.length/2; i++) {
                texcoords[2*i+0] /= width;
                texcoords[2*i+1] /= height;
            }
        }

        public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
        {
            gl.glColor(color);
            gl.glNormal3f(0,0,1);

            VisFont.this.texture.bind(gl);
            gl.gldBind(GL.VBO_TYPE_VERTEX,    vertsID,     verts.length / 2,     2, verts);
            gl.gldBind(GL.VBO_TYPE_TEX_COORD, texCoordsID, texcoords.length / 2, 2, texcoords);

            gl.glDrawArrays(GL.GL_QUADS, 0, verts.length / 2);

            gl.gldUnbind(GL.VBO_TYPE_VERTEX,    vertsID);
            gl.gldUnbind(GL.VBO_TYPE_TEX_COORD, texCoordsID);

            VisFont.this.texture.unbind(gl);
        }
    }
}
