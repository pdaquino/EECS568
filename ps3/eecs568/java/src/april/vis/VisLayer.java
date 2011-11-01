package april.vis;

import java.util.*;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import javax.swing.*;

/** A VisLayer provides information about how to render a VisWorld. A
 * VisLayer is owned by a single VisCanvas.
 **/
public class VisLayer implements Comparable<VisLayer>, VisSerializable
{
    public boolean enabled = true;

    public boolean clearDepth = true;

    // The objects in the world.
    public VisWorld world;

    // low numbers draw before big numbers
    public int drawOrder;

    // The color of the layer before any objects are drawn
    public Color backgroundColor = new Color(0, 0, 0, 255); //Color.black;

    // Where are the lights that illuminate the objects in the world?
    public ArrayList<VisLight> lights = new ArrayList<VisLight>();

    // Where is the camera?
    public VisCameraManager cameraManager = new DefaultCameraManager();

    // How is the layer positioned with respect to the viewport?
    public VisLayerManager layerManager = new DefaultLayerManager();

    // How do we determine what point to translate/rotate around in
    // response to a user interface click?
    public VisManipulationManager manipulationManager = new DefaultManipulationManager();

    // synchronize on list before accessing. Invariant: event handlers
    // are sorted by priority, higher priority handlers first. Use addEventHandler!
    public ArrayList<VisEventHandler> eventHandlers = new ArrayList<VisEventHandler>();

    VisEventHandler popupMenu = new DefaultPopupMenu(this);

    public int popupBackgroundColors[] = new int[] { 0x000000, 0x808080, 0xffffff };

    public VisLayer(VisWorld vw)
    {
        this.world = vw;

        lights.add(new VisLight(new float[] { 100f, 150f, 120f, 1.0f },
                                new float[] { .4f, .4f, .4f, 1.0f},
                                new float[] { .8f, .8f, .8f, 1.0f },
                                new float[] { .5f, .5f, .5f, 1.0f}));


        lights.add(new VisLight(new float[] { -100f, -150f, 120f, 1.0f },
                                new float[] { .1f, .1f, .1f, 1.0f},
                                new float[] { .1f, .1f, .1f, 1.0f },
                                new float[] { .5f, .5f, .5f, 1.0f}));

        addEventHandler(new DefaultEventHandler());
        addEventHandler(popupMenu);
    }

    public void addEventHandler(VisEventHandler eh)
    {
        synchronized(eventHandlers) {
            eventHandlers.add(eh);
            Collections.sort(eventHandlers, new EventHandlerComparator());
        }
    }

    class EventHandlerComparator implements Comparator<VisEventHandler>
    {
        public int compare(VisEventHandler a, VisEventHandler b)
        {
            return a.getPriority() - b.getPriority();
        }
    }

    public int compareTo(VisLayer vl)
    {
        return drawOrder - vl.drawOrder;
    }

    public void populatePopupMenu(JPopupMenu jmenu)
    {
        // background color
        if (true) {
            JMenu jm = new JMenu("Background Color");
            JRadioButtonMenuItem jmis[] = new JRadioButtonMenuItem[popupBackgroundColors.length];
            ButtonGroup group = new ButtonGroup();

            for (int i = 0; i < jmis.length; i++) {
                jmis[i] = new JRadioButtonMenuItem(String.format("%08x", popupBackgroundColors[i]));
                group.add(jmis[i]);
                jm.add(jmis[i]);

                jmis[i].addActionListener(new ActionListener() {
                    public void actionPerformed(ActionEvent e) {
                        JRadioButtonMenuItem jmi = (JRadioButtonMenuItem) e.getSource();
                        backgroundColor = new Color(Integer.parseInt(jmi.getText(), 16));
                    }
                });
            }

            int bestIndex = 0;
            int v = backgroundColor.getRGB() & 0xffffff;
            for (int i = 0; i < popupBackgroundColors.length; i++) {
                if (Math.abs(popupBackgroundColors[i] - v) < Math.abs(popupBackgroundColors[bestIndex] - v))
                    bestIndex = i;
            }

            jmis[bestIndex].setSelected(true);

            jmenu.add(jm);
        }

        // buffer menu
        if (true) {
            JMenu jm = new JMenu("Buffers");

            synchronized(world.buffers) {

                JCheckBoxMenuItem jmis[] = new JCheckBoxMenuItem[world.buffers.size()];

                for (int i = 0; i < world.buffers.size(); i++) {
                    VisWorld.Buffer vb = world.buffers.get(i);

                    jmis[i] = new JCheckBoxMenuItem(vb.getName());
                    jmis[i].setSelected(vb.isEnabled());

                    jmis[i].addActionListener(new ActionListener() {
                        public void actionPerformed(ActionEvent e) {
                            JCheckBoxMenuItem jmi = (JCheckBoxMenuItem) e.getSource();
                            String bufferName = jmi.getText();

                            VisWorld.Buffer vb = world.getBuffer(bufferName);
                            vb.setEnabled(!vb.isEnabled());
                        }
                    });

                    jm.add(jmis[i]);
                }
            }

            if (false) {
                jm.addSeparator();

                JMenuItem jmi = new JMenuItem("Detatch...");
                // XXX TODO
                jm.add(jmi);
            }

            jmenu.add(jm);
        }
    }

    /** For use only be serialization **/
    public VisLayer(ObjectReader obj)
    {
    }

    public void writeObject(ObjectWriter outs) throws IOException
    {
        outs.writeBoolean(enabled);
        outs.writeBoolean(clearDepth);
        outs.writeObject(world);
        outs.writeInt(drawOrder);
        outs.writeColor(backgroundColor);

        outs.writeInt(lights.size());
        for (VisLight light : lights)
            outs.writeObject(light);

        outs.writeObject(cameraManager);
        outs.writeObject(layerManager);
        outs.writeObject(manipulationManager);

        outs.writeInt(eventHandlers.size());
        for (VisEventHandler eh : eventHandlers)
            outs.writeObject(eh);

        outs.writeObject(popupMenu);

        outs.writeInts(popupBackgroundColors);
    }

    public void readObject(ObjectReader ins) throws IOException
    {
        enabled = ins.readBoolean();
        clearDepth = ins.readBoolean();
        world = (VisWorld) ins.readObject();
        drawOrder = ins.readInt();
        backgroundColor = ins.readColor();

        int n = ins.readInt();
        for (int i = 0; i < n; i++)
            lights.add((VisLight) ins.readObject());

        cameraManager = (VisCameraManager) ins.readObject();
        layerManager = (VisLayerManager) ins.readObject();
        manipulationManager = (VisManipulationManager) ins.readObject();

        n = ins.readInt();
        for (int i = 0; i < n; i++)
            addEventHandler((VisEventHandler) ins.readObject());

        popupMenu = (VisEventHandler) ins.readObject();

        popupBackgroundColors = ins.readInts();
    }

}
