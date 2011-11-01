package april.vis;

import java.awt.*;
import java.io.*;
import java.util.*;

public class ObjectReader
{
    DataInputStream ins;

    HashMap<Long, Object> objectIDs = new HashMap<Long, Object>();

    public ObjectReader(DataInputStream ins)
    {
        this.ins = ins;
    }

    public boolean readBoolean() throws IOException
    {
        return ins.readBoolean();
    }

    public int readInt() throws IOException
    {
        return ins.readInt();
    }

    public int readByte() throws IOException
    {
        return ins.readByte();
    }

    public long readLong() throws IOException
    {
        return ins.readLong();
    }

    public float readFloat() throws IOException
    {
        return ins.readFloat();
    }

    public double readDouble() throws IOException
    {
        return ins.readDouble();
    }

    public String readUTF() throws IOException
    {
        return ins.readUTF();
    }

    public Color readColor() throws IOException
    {
        int r = ins.readByte() & 0xff;
        int g = ins.readByte() & 0xff;
        int b = ins.readByte() & 0xff;
        int a = ins.readByte() & 0xff;

        return new Color(r, g, b, a);
    }

    public float[] readFloats() throws IOException
    {
        int len = readInt();
        if (len < 0)
            return null;

        float v[] = new float[len];
        for (int i = 0; i < v.length; i++)
            v[i] = readFloat();
        return v;
    }

    public double[] readDoubles() throws IOException
    {
        int len = readInt();
        if (len < 0)
            return null;

        double v[] = new double[len];
        for (int i = 0; i < v.length; i++)
            v[i] = readDouble();
        return v;
    }

    public int[] readInts() throws IOException
    {
        int len = readInt();
        if (len < 0)
            return null;

        int v[] = new int[len];
        for (int i = 0; i < v.length; i++)
            v[i] = readInt();
        return v;
    }

    public double[][] readDoubleMatrix() throws IOException
    {
        int a = readInt();
        if (a < 0)
            return null;

        int b = readInt();

        double v[][] = new double[a][b];
        for (int i = 0; i < v.length; i++)
            for (int j = 0; j < v[0].length; j++)
                v[i][j] = ins.readDouble();

        return v;
    }

    public Object readObject() throws IOException
    {
        String cls = ins.readUTF();

        long id = ins.readLong();

        int mode = ins.readByte()&0xff;

        System.out.println("readObject: "+cls+" "+id+" "+mode);

        switch (mode) {
            case 0: {
                assert(objectIDs.get(id) != null);
                return objectIDs.get(id);
            }

            case 1: {
                Object obj = createObject(cls);
                objectIDs.put(id, obj);
                ((VisSerializable) obj).readObject(this);
                return obj;
            }

            case 2: {
                Object obj = createObjectEmpty(cls);
                objectIDs.put(id, obj);
                return obj;
            }

            default: {
                assert(false);
                break;
            }
        }

        assert(false);
        return null;
    }

    public Object createObject(String className)
    {
        try {
            Class cls = Class.forName(className);
            Object o = cls.getConstructor(this.getClass()).newInstance(new Object[] { null } );
            return o;
        } catch (Exception ex) {
            System.out.println("ReflectUtil.createObject ex: "+ex);
            ex.printStackTrace();
            return null;
        }
    }

    public Object createObjectEmpty(String className)
    {
        try {
            Class cls = Class.forName(className);
            Object o = cls.getConstructor().newInstance();
            return o;
        } catch (Exception ex) {
            System.out.println("ReflectUtil.createObject ex: "+ex);
            ex.printStackTrace();
            return null;
        }
    }

}
