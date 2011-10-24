package april.vis;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;
import java.util.zip.*;
import javax.swing.*;
import javax.imageio.*;

import april.jmat.*;
import april.jmat.geom.*;

import java.util.concurrent.atomic.*;

/** A VisCanvas coordinates the rendering and event handling for a
 * collection of VisLayers in a single JComponent.
 **/
public class VisCanvas extends JComponent
{
    GLManager glManager = GLManager.getSingleton();
    BufferedImage im = null;
    RedrawTask redrawTask = new RedrawTask();

    int frameCounter;

    long canvasId;
    static AtomicLong nextId = new AtomicLong(1);

    // protected by synchronizing on 'layers'
    ArrayList<VisLayer> layers = new ArrayList<VisLayer>();

    EventHandler eh = new EventHandler();

    RenderInfo lastRenderInfo;

    public int popupFrameRates[] = new int[] { 1, 5, 10, 20, 30, 60, 100 };
    public int targetFrameRate = 20;

    // a list of open movies.
    ArrayList<Movie> movies = new ArrayList<Movie>();
    Movie popupMovie;

    // time-averaged frame rate (empirically measured), but
    // initialized with a good guess.
    double fpsDt = 1.000 / targetFrameRate;

    public class Movie
    {
        boolean autoframes;
        GZIPOutputStream outs;

        Movie(GZIPOutputStream outs, boolean autoframes)
        {
            this.autoframes = autoframes;
            this.outs = outs;
        }

        synchronized public void addFrame()
        {
            drawSync();
            addFrame(im);
        }

        synchronized public void close() throws IOException
        {
            synchronized(movies) {
                movies.remove(this);
            }

            outs.close();
        }

        synchronized void addFrame(BufferedImage upsideDownImage)
        {
            BufferedImage im = upsideDownImage;

            int width = im.getWidth(), height = im.getHeight();

            int idata[] = ((DataBufferInt) (im.getRaster().getDataBuffer())).getData();
            byte bdata[] = new byte[width*height*3];
            int bdatapos = 0;

            for (int y = height-1; y >= 0; y--) {
                for (int x = 0; x < width; x++) {
                    int rgb = idata[y*width+x];
                    bdata[bdatapos++] = (byte) ((rgb>>16)&0xff);
                    bdata[bdatapos++] = (byte) ((rgb>>8)&0xff);
                    bdata[bdatapos++] = (byte) ((rgb>>0)&0xff);
                }
            }

            try {
                String hdr = "";
                hdr += String.format("# mtime=%d\n", System.currentTimeMillis());
                hdr += String.format("P6 %d %d %d\n", width, height, 255);

                outs.write(hdr.getBytes());
                outs.write(bdata);
                outs.flush();
            } catch (IOException ex) {
                System.out.println("Error writing movie: "+ex);
            }
        }
    }

    public static class RenderInfo
    {
        // The layers, in the order that they were rendered.
        public ArrayList<VisLayer> layers = new ArrayList<VisLayer>();

        // The position of the layers when they were rendered.
        public HashMap<VisLayer, int[]> layerPositions = new HashMap<VisLayer, int[]>();

        public HashMap<VisLayer, VisCameraManager.CameraPosition> cameraPositions = new HashMap<VisLayer, VisCameraManager.CameraPosition>();
    }

    // XXX support as many layers as desired
    public VisCanvas(VisLayer layer)
    {
        canvasId = nextId.getAndIncrement();

        addComponentListener(new MyComponentListener());

        addLayer(layer);

        addMouseMotionListener(eh);
        addMouseListener(eh);
        addMouseWheelListener(eh);
        addKeyListener(eh);

        setFocusTraversalKeysEnabled(false);

        new RepaintThread().start();
    }

    public void addLayer(VisLayer layer)
    {
        layer.canvas = this;
        layers.add(layer);
    }

    class RepaintThread extends Thread
    {
        public RepaintThread()
        {
            setDaemon(true);
        }

        public void run()
        {
            while (true) {
                try {
                    Thread.sleep(1000 / targetFrameRate);
                } catch (InterruptedException ex) {
                    System.out.println("ex: "+ex);
                }
                glManager.add(redrawTask);
            }
        }
    }

    class MyComponentListener extends ComponentAdapter
    {
        public void componentResized(ComponentEvent e)
        {
            draw();
        }

        public void componentShown(ComponentEvent e)
        {
            draw();
        }
    }

    // This task serves as a synchronization barrier, which we use
    // perform synchronous frame updates.
    class SyncTask implements GLManager.Task
    {
        Object o = new Object();

        public void run()
        {
            synchronized(o) {
                o.notifyAll();
            }
        }
    }

    class RedrawTask implements GLManager.Task
    {
        GL gl;
        int fboId;
        int fboWidth, fboHeight;

        long lastDrawTime;

        public void run()
        {
            // GL object must be created from the GLManager thread.
            if (gl == null) {
                gl = new GL();
            }

            int width = getWidth();
            int height = getHeight();

            if (!VisCanvas.this.isVisible() || width==0 || height==0 )
                return;

            // XXX Should we only reallocate an FBO if our render size
            // has gotten bigger? If we do this, we should modify
            // getFrame() so that we don't read parts of the image
            // that we don't need.
            if (fboId <= 0 || fboWidth != width || fboHeight != height) {
                if (fboId > 0) {
                    gl.frameBufferDestroy(fboId);
                    fboId = 0;
                }

                fboId = gl.frameBufferCreate(width, height);
                if (fboId < 0) {
                    System.out.println("Failed to create frame buffer. "+width+"\n");
                    return;
                }
                fboWidth = width;
                fboHeight = height;
            }

            gl.frameBufferBind(fboId);

            gl.gldFrameBegin(canvasId);

            ///////////////////////////////////////////////
            // Begin GL Rendering
            int viewport[] = new int[] { 0, 0, getWidth(), getHeight() };

            gl.glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);

            gl.glEnable(GL.GL_NORMALIZE);
            gl.glEnable(GL.GL_LIGHTING);

            gl.glLightModeli(GL.GL_LIGHT_MODEL_TWO_SIDE, GL.GL_TRUE);
            gl.glEnable(GL.GL_COLOR_MATERIAL);
            gl.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE);
            gl.glMaterialf(GL.GL_FRONT_AND_BACK, GL.GL_SHININESS, 0);
            gl.glMaterialfv(GL.GL_FRONT_AND_BACK, GL.GL_SPECULAR, new float[] {.1f, .1f, .1f, .1f});

            gl.glEnable(GL.GL_DEPTH_TEST);
            gl.glDepthFunc(GL.GL_LEQUAL);

            gl.glEnable(GL.GL_BLEND);
            gl.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA);

            gl.glPolygonMode(GL.GL_FRONT, GL.GL_FILL);
            gl.glPolygonMode(GL.GL_BACK, GL.GL_FILL);

            gl.glDisable(GL.GL_LINE_STIPPLE);

            gl.glShadeModel(GL.GL_SMOOTH);

/*
            gl.glEnable(GL.GL_POINT_SMOOTH);
            gl.glHint(GL.GL_POINT_SMOOTH_HINT, GL.GL_NICEST);

            gl.glEnable(GL.GL_LINE_SMOOTH);
            gl.glHint(GL.GL_LINE_SMOOTH_HINT, GL.GL_NICEST);

            gl.glEnable(GL.GL_POLYGON_SMOOTH);
            gl.glHint(GL.GL_POLYGON_SMOOTH_HINT, GL.GL_NICEST);
*/

            gl.glEnable(GL.GL_SCISSOR_TEST);

            long mtime = System.currentTimeMillis();

            // rinfo records where we rendered everything, which we'll
            // need in order to properly handle events.
            RenderInfo rinfo = new RenderInfo();
            synchronized(layers) {
                rinfo.layers.addAll(layers);
            }

            Collections.sort(rinfo.layers);

            for (VisLayer layer : rinfo.layers) {

                if (!layer.enabled)
                    continue;

                gl.glClearDepth(1.0);
                gl.glClearColor(layer.backgroundColor.getRed()/255f,
                                layer.backgroundColor.getGreen()/255f,
                                layer.backgroundColor.getBlue()/255f,
                                layer.backgroundColor.getAlpha()/255f);

                int layerPosition[] = layer.layerManager.getLayerPosition(VisCanvas.this, viewport, layer, mtime);
                rinfo.layerPositions.put(layer, layerPosition);

                gl.glScissor(layerPosition[0], layerPosition[1], layerPosition[2], layerPosition[3]);
                gl.glViewport(layerPosition[0], layerPosition[1], layerPosition[2], layerPosition[3]);

                gl.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT);

                ///////////////////////////////////////////////////////
                // set up lighting

                // The position of lights is transformed by the
                // current model view matrix, thus we load the
                // identity matrix before configuring the lights.
                gl.glMatrixMode(GL.GL_MODELVIEW);
                gl.glLoadIdentity();

                for (int i = 0; i < layer.lights.size(); i++) {
                    VisLight light = layer.lights.get(i);
                    gl.glLightfv(GL.GL_LIGHT0 + i, GL.GL_POSITION, light.position);
                    gl.glLightfv(GL.GL_LIGHT0 + i, GL.GL_AMBIENT, light.ambient);
                    gl.glLightfv(GL.GL_LIGHT0 + i, GL.GL_DIFFUSE, light.diffuse);
                    gl.glLightfv(GL.GL_LIGHT0 + i, GL.GL_SPECULAR, light.specular);

                    gl.glEnable(GL.GL_LIGHT0 + i);
                }

                // position the camera
                VisCameraManager.CameraPosition cameraPosition = layer.cameraManager.getCameraPosition(VisCanvas.this,
                                                                                                       viewport,
                                                                                                       layerPosition,
                                                                                                       layer,
                                                                                                       mtime);
                rinfo.cameraPositions.put(layer, cameraPosition);

                gl.glMatrixMode(GL.GL_PROJECTION);
                gl.glLoadIdentity();
                gl.glMultMatrix(cameraPosition.getProjectionMatrix());

                gl.glMatrixMode(GL.GL_MODELVIEW);
                gl.glLoadIdentity();
                gl.glMultMatrix(cameraPosition.getModelViewMatrix());

                // draw the objects
                layer.world.render(VisCanvas.this, layer, rinfo, gl);

                // undo our lighting
                for (int i = 0; i < layer.lights.size(); i++) {
                    gl.glDisable(GL.GL_LIGHT0 + i);
                }
            }

            gl.gldFrameEnd(canvasId);

            int err = gl.glGetError();
            if (err != 0)
                System.out.printf("glGetError: %d\n", err);

            im = gl.getImage(im);

            repaint();
            lastRenderInfo = rinfo;

            synchronized(movies) {
                for (Movie m : movies) {
                    if (m.autoframes)
                        m.addFrame(im);
                }
            }

            if (true) {
                long now = System.currentTimeMillis();
                double dt = (now - lastDrawTime) / 1000.0;

                dt = Math.max(0, Math.min(2, dt));

                // larger alpha = favor existing estimate
                double fpsAlpha = 1.0 - dt;
                fpsAlpha = Math.max(0, Math.min(1, fpsAlpha));

                if (fpsDt == 0)
                    fpsAlpha = 0;
                fpsDt = fpsDt * fpsAlpha + dt * (1.0 - fpsAlpha);

                lastDrawTime = now;
            }

            frameCounter++;
        }
    }

    public double getMeasuredFPS()
    {
        return 1.0 / fpsDt;
    }

    /** Caution: can return null. **/
    public RenderInfo getLastRenderInfo()
    {
        return lastRenderInfo;
    }

    /** Schedule a repainting operation as soon as possible, even if
     * the target frame rate is exceeded.
     **/
    public void draw()
    {
        glManager.add(redrawTask);
    }

    public void paintComponent(Graphics _g)
    {
        Graphics2D g = (Graphics2D) _g;

        if (im != null) {
            g.translate(0, getHeight());
            g.scale(1, -1);
            g.drawImage(im, 0, 0, null);
        }
    }

    class EventHandler implements MouseMotionListener, MouseListener, MouseWheelListener, KeyListener
    {
        VisLayer mousePressedLayer;

        public void keyPressed(KeyEvent e)
        {
        }

        public void keyReleased(KeyEvent e)
        {
        }

        public void keyTyped(KeyEvent e)
        {
        }

        public void mouseWheelMoved(MouseWheelEvent e)
        {
            dispatchMouseEvent(e);
        }

        public void mouseDragged(MouseEvent e)
        {
            dispatchMouseEvent(e);
        }

        public void mouseMoved(MouseEvent e)
        {
            dispatchMouseEvent(e);
        }

        public void mousePressed(MouseEvent e)
        {
            dispatchMouseEvent(e);
        }

        public void mouseReleased(MouseEvent e)
        {
            dispatchMouseEvent(e);
        }

        public void mouseClicked(MouseEvent e)
        {
            dispatchMouseEvent(e);
        }

        public void mouseEntered(MouseEvent e)
        {
            dispatchMouseEvent(e);
            requestFocus();
        }

        public void mouseExited(MouseEvent e)
        {
            dispatchMouseEvent(e);
        }

        void dispatchMouseEvent(MouseEvent e)
        {
            RenderInfo rinfo = lastRenderInfo;
            if (rinfo == null)
                return;

            int ex = e.getX();
            int ey = getHeight() - e.getY();

            if (e.getID() == MouseEvent.MOUSE_DRAGGED || e.getID() == MouseEvent.MOUSE_RELEASED) {
                if (mousePressedLayer != null)
                    dispatchMouseEventToLayer(VisCanvas.this, mousePressedLayer, rinfo,
                                              rinfo.cameraPositions.get(mousePressedLayer).computeRay(ex, ey), e);

                return;
            }

            for (int lidx = rinfo.layers.size()-1; lidx >= 0; lidx--) {
                VisLayer layer = rinfo.layers.get(lidx);
                if (!layer.enabled)
                    continue;

                int pos[] = rinfo.layerPositions.get(layer);

                GRay3D ray = rinfo.cameraPositions.get(layer).computeRay(ex, ey);

                if (ex >= pos[0] && ey >= pos[1] &&
                    ex < pos[0]+pos[2] && ey < pos[1]+pos[3]) {

                    boolean handled = dispatchMouseEventToLayer(VisCanvas.this, layer, rinfo, ray, e);

                    if (e.getID() == MouseEvent.MOUSE_PRESSED) {
                        if (handled)
                            mousePressedLayer = layer;
                        else
                            mousePressedLayer = null;
                    }

                    if (handled)
                        return;
                }
            }
        }

        boolean dispatchMouseEventToLayer(VisCanvas vc, VisLayer layer, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            boolean handled = false;

            synchronized (layer.eventHandlers) {
                for (VisEventHandler eh : layer.eventHandlers) {

                    switch (e.getID()) {
                        case MouseEvent.MOUSE_PRESSED:
                            mousePressedLayer = layer;
                            handled = eh.mousePressed(VisCanvas.this, layer, rinfo, ray, e);
                            break;
                        case MouseEvent.MOUSE_RELEASED:
                            handled = eh.mouseReleased(VisCanvas.this, layer, rinfo, ray, e);
                            break;
                        case MouseEvent.MOUSE_CLICKED:
                            handled = eh.mouseClicked(VisCanvas.this, layer, rinfo, ray, e);
                            break;
                        case MouseEvent.MOUSE_DRAGGED:
                            handled = eh.mouseDragged(VisCanvas.this, layer, rinfo, ray, e);
                            break;
                        case MouseEvent.MOUSE_MOVED:
                            handled = eh.mouseMoved(VisCanvas.this, layer, rinfo, ray, e);
                            break;
                        case MouseEvent.MOUSE_WHEEL:
                            handled = eh.mouseWheel(VisCanvas.this, layer, rinfo, ray, (MouseWheelEvent) e);
                            break;
                        case MouseEvent.MOUSE_ENTERED:
                            handled = false;
                            break;
                        case MouseEvent.MOUSE_EXITED:
                            handled = false;
                            break;
                        default:
                            System.out.println("Unhandled mouse event id: "+e.getID());
                            handled = false;
                            break;
                    }

                    if (handled)
                        break;
                }
            }

            return handled;
        }
    }

    public void drawSync()
    {
        SyncTask st = new SyncTask();

        synchronized(st.o) {
            glManager.add(redrawTask); // ensure we're queued
            glManager.add(st);

            try {
                st.o.wait();
            } catch (InterruptedException ex) {
                System.out.println("VisCanvas: drawSync interrupted!");
            }
        }
    }

    /** Forces a synchronous redraw and then draws. **/
    public void writeScreenShot(File file, String format)
    {
        drawSync();

        BufferedImage thisim = im;

        // Our image will be upside down. let's flip it.
        if (true) {
            int height = thisim.getHeight();
            int width = thisim.getWidth();
            int stride = thisim.getWidth();

            int imdata[] = ((DataBufferInt) (thisim.getRaster().getDataBuffer())).getData();

            BufferedImage thisim2 = new BufferedImage(width, height, thisim.getType());
            int imdata2[] = ((DataBufferInt) (thisim2.getRaster().getDataBuffer())).getData();

            for (int y = 0; y < height; y++)
                System.arraycopy(imdata, y*stride, imdata2, (height-1-y)*stride, stride);

            thisim = thisim2;
        }

        try {
            ImageIO.write(thisim, format, file);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
            return;
        }

        System.out.println("Screen shot written to "+file.getPath());
        return;
    }

    /** There are two ways to make movies which differ in when frames
     * are added to the movie. In 'autoframes' mode, every rendered
     * frame is added to the movie. In manual mode, frames are added
     * programmatically by calls to movieAddFrame **/
    public Movie movieCreate(String path, boolean autoframes) throws IOException
    {
        Movie m = new Movie(new GZIPOutputStream(new FileOutputStream(path)), autoframes);

        synchronized(movies) {
            movies.add(m);
        }

        return m;
    }

    public void populatePopupMenu(JPopupMenu jmenu)
    {
        if (true) {
            JMenuItem jmi = new JMenuItem("Save screenshot (.png)");
            jmi.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {

                    Calendar c = new GregorianCalendar();

                    String s = String.format("%4d%02d%02d_%02d%02d%02d_%03d", c.get(Calendar.YEAR),
                                             c.get(Calendar.MONTH)+1,
                                             c.get(Calendar.DAY_OF_MONTH),
                                             c.get(Calendar.HOUR_OF_DAY),
                                             c.get(Calendar.MINUTE),
                                             c.get(Calendar.SECOND),
                                             c.get(Calendar.MILLISECOND)
                        );

                    String path = "p"+s+".png";

                    writeScreenShot(new File(path), "png");
                }
            });
            jmenu.add(jmi);
        }

        if (true) {
            JMenuItem jmi = new JMenuItem("Save scene (.vsc)");
            jmi.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
// XXX
                }
            });
            jmenu.add(jmi);
        }

        if (popupMovie == null) {
            JMenuItem jmi = new JMenuItem("Record Movie");
            jmi.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    Calendar c = new GregorianCalendar();
                    String s = String.format("%4d%02d%02d_%02d%02d%02d_%03d.ppms.gz", c.get(Calendar.YEAR),
                                             c.get(Calendar.MONTH)+1,
                                             c.get(Calendar.DAY_OF_MONTH),
                                             c.get(Calendar.HOUR_OF_DAY),
                                             c.get(Calendar.MINUTE),
                                             c.get(Calendar.SECOND),
                                             c.get(Calendar.MILLISECOND)
                        );

                    try {
                        popupMovie = movieCreate(s, true);
                    } catch (IOException ex) {
                        System.out.println("ex: "+ex);
                    }
                }
            });
            jmenu.add(jmi);
        } else {
            JMenuItem jmi = new JMenuItem("Stop Recording Movie");
            jmi.addActionListener(new ActionListener() {
                public void actionPerformed(ActionEvent e) {
                    try {
                        popupMovie.close();

                    } catch (IOException ex) {
                        System.out.println("ex: "+ex);
                    }
                }
            });
            jmenu.add(jmi);
        }


        if (true) {
            JMenu jm = new JMenu("Max Frame Rate");
            JRadioButtonMenuItem jmis[] = new JRadioButtonMenuItem[popupFrameRates.length];
            ButtonGroup group = new ButtonGroup();

            for (int i = 0; i < jmis.length; i++) {
                jmis[i] = new JRadioButtonMenuItem(""+popupFrameRates[i]);
                group.add(jmis[i]);
                jm.add(jmis[i]);

                jmis[i].addActionListener(new ActionListener() {
                    public void actionPerformed(ActionEvent e) {
                        JRadioButtonMenuItem jmi = (JRadioButtonMenuItem) e.getSource();
                        VisCanvas.this.targetFrameRate = Integer.parseInt(jmi.getText());
                    }
                });
            }

            int bestIndex = 0;
            for (int i = 0; i < popupFrameRates.length; i++) {
                if (Math.abs(popupFrameRates[i] - targetFrameRate) < Math.abs(popupFrameRates[bestIndex] - targetFrameRate))
                    bestIndex = i;
            }

            jmis[bestIndex].setSelected(true);

            jmenu.add(jm);

        }
    }
}

