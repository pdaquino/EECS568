package april.vis;

import java.util.*;
import java.awt.*;

import april.util.*;

/** A VisWorld represents a scene of objects. This scene could be
 * rendered by one or more VisLayers.
 **/
public class VisWorld
{
    static boolean debug = EnvUtil.getProperty("vis.debug", false);

    // synchrronize on 'buffers' before accessing.
    ArrayList<Buffer> buffers = new ArrayList<Buffer>();

    // 'bufferMap' is protected by synchronizing on 'buffers'.
    HashMap<String, Buffer> bufferMap = new HashMap<String, Buffer>();

    public class Buffer implements Comparable<Buffer>
    {
        // contents of 'front' and 'back' are protected by synchronizing on the buffer.
        protected ArrayList<VisObject> back  = new ArrayList<VisObject>();
        protected ArrayList<VisObject> front = new ArrayList<VisObject>();

        int drawOrder = -1;
        String name;
        boolean enabled = true;

        Buffer(String name)
        {
            this.name = name;
        }

        public String getName()
        {
            return name;
        }

        public synchronized void addBack(VisObject vo)
                                 {
                                     back.add(vo);
                                 }

        public synchronized void clear()
                                 {
                                     back.clear();
                                     front.clear();
                                 }

        public synchronized void addFront(VisObject vo)
                                 {
                                     front.add(vo);
                                 }

        public synchronized void swap()
                                 {
                                     front = back;
                                     // don't recycle: a previous front buffer
                                     // could still have a reference somewhere.
                                     back = new ArrayList<VisObject>();
                                 }

        public int compareTo(Buffer b)
        {
            return drawOrder - b.drawOrder;
        }

        public void setDrawOrder(int order)
        {
            if (order != this.drawOrder) {
                this.drawOrder = order;
                synchronized(buffers) {
                    Collections.sort(buffers);
                }
            }
        }

        public void setEnabled(boolean b)
        {
            enabled = b;
        }

        public boolean isEnabled()
        {
            return enabled;
        }
    }

    public Buffer getBuffer(String name)
    {
        Buffer b = bufferMap.get(name);
        if (b == null) {
            b = new Buffer(name);
            synchronized(buffers) {
                bufferMap.put(name, b);
                buffers.add(b);
                Collections.sort(buffers);
            }
        }

        return b;
    }

    public void render(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GL gl)
    {
        synchronized(buffers) {
            for (Buffer b : buffers) {
                if (!b.isEnabled())
                    continue;
                synchronized(b) {
                    for (VisObject vo : b.front) {
                        if (vo != null)
                            vo.render(vc, layer, rinfo, gl);
                    }
                }
            }
        }
    }
}
