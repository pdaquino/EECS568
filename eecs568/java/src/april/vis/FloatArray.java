package april.vis;

public final class FloatArray
{
    float data[] = new float[16];
    int  pos; // index of next index to write to.

    public void ensureSpace(int additionalCapacity)
    {
        if (pos + additionalCapacity < data.length)
            return;

        int newsize = 2 * data.length;

        while (newsize < pos + additionalCapacity)
            newsize *= 2;

        float f[] = new float[newsize];
        System.arraycopy(data, 0, f, 0, pos);
        data = f;
    }

    public void add(float f)
    {
        ensureSpace(1);
        data[pos++] = f;
    }

    public float[] getData()
    {
        return data;
    }

    public int size()
    {
        return pos;
    }
}
