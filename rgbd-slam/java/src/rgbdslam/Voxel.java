package rgbdslam;

public class Voxel
{
    int aTot = 0;
    int rTot = 0;
    int gTot = 0;
    int bTot = 0;

    int hitCount = 0;

    public Voxel()
    {
        this(0);
    }

    public Voxel(int argb)
    {
        addSample(argb);
    }

    public void addSample(int argb)
    {
        hitCount++;
        aTot += ((argb >> 24) & 0xff);
        rTot += ((argb >> 16) & 0xff);
        gTot += ((argb >> 8) & 0xff);
        bTot += (argb & 0xff);
    }

    public int getARGB()
    {
        if (hitCount == 0)
            return 0;
        int a = aTot/hitCount;
        int r = rTot/hitCount;
        int g = gTot/hitCount;
        int b = bTot/hitCount;
        return ((a & 0xff) << 24) |
               ((r & 0xff) << 16) |
               ((g & 0xff) << 8)  |
               (b & 0xff);
    }

    public Voxel merge(Voxel v)
    {
        Voxel voxel = new Voxel();
        voxel.aTot = aTot + v.aTot;
        voxel.rTot = rTot + v.rTot;
        voxel.gTot = gTot + v.gTot;
        voxel.bTot = bTot + v.bTot;

        return voxel;
    }

    public Voxel copy()
    {
        Voxel v = new Voxel();
        v.aTot = aTot;
        v.rTot = rTot;
        v.gTot = gTot;
        v.bTot = bTot;
        v.hitCount = hitCount;

        return v;
    }
}
