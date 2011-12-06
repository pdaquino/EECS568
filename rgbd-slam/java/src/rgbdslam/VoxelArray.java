package rgbdslam;

import java.util.*;
import java.awt.*;

import april.jmat.*;
import april.vis.*;

import kinect.*;

/** A structure to store colored point cloud data
 *  in voxel space of arbitrary resolution.
 */
public class VoxelArray
{
    HashMap<VoxelKey, Voxel> voxels = new HashMap<VoxelKey, Voxel>();
    double resolution;

    public VoxelArray(double resolution_)
    {
        resolution = resolution_;
    }

    public int size()
    {
        return voxels.keySet().size();
    }

    public void voxelizePointCloud(ColorPointCloud cpc)
    {
        for (int i = 0; i < cpc.numPoints(); i++) {
            double[] xyz = cpc.points.get(i);
            //double[] xyz = LinAlg.transform(rbt, cpc.points.get(i));
            VoxelKey vk = new VoxelKey(xyz);
            if (!voxels.containsKey(vk))
                voxels.put(vk, new Voxel(cpc.colors.get(i)));
            else
                voxels.get(vk).addSample(cpc.colors.get(i));
        }
    }

    public VzPoints getPointCloud()
    {
        ArrayList<double[]> points = new ArrayList<double[]>();
        VisColorData vcd = new VisColorData();
        for (VoxelKey vk: voxels.keySet()) {
            Voxel voxel = voxels.get(vk);
            double x = vk.xyz[0]*resolution + 0.5*resolution;
            double y = vk.xyz[1]*resolution + 0.5*resolution;
            double z = vk.xyz[2]*resolution + 0.5*resolution;
            points.add(new double[] {x,y,z});
            vcd.add(voxel.getABGR());
        }

        return new VzPoints(new VisVertexData(points),
                            new VzPoints.Style(vcd, 2));
    }

    public ArrayList<VisChain> getBoxes()
    {
        double[][] scale = LinAlg.scale(resolution);

        ArrayList<VisChain> boxes = new ArrayList<VisChain>();
        for (VoxelKey vk: voxels.keySet()) {
            Voxel voxel = voxels.get(vk);
            VzBox box = new VzBox(new VzMesh.Style(new Color(voxel.getARGB())));
            double x = vk.xyz[0]*resolution + 0.5*resolution;
            double y = vk.xyz[1]*resolution + 0.5*resolution;
            double z = vk.xyz[2]*resolution + 0.5*resolution;
            double[][] translate = LinAlg.translate(x, y, z);
            boxes.add(new VisChain(translate,
                                   scale,
                                   box));
        }

        return boxes;
    }

    // XXX Needs verification. Unit tests...go!
    /** Given another voxel array and a rigid body
     *  transform between our local coordinates and
     *  the other array's coordinates (rbt maps a
     *  coodinate in va-local to our local), make
     *  a merged voxel array
     **/
    public VoxelArray merge(VoxelArray va, Matrix rbt)
    {
        assert (va.resolution == resolution);

        // Transform the other voxels to our space
        for (VoxelKey vk: va.voxels.keySet()) {
            VoxelKey vk_xform = vk.transform(rbt);
            if (!voxels.containsKey(vk_xform)) {
                voxels.put(vk_xform, va.voxels.get(vk).copy());
            } else {
                voxels.get(vk_xform).merge(va.voxels.get(vk).copy());
            }
        }

        return this;
    }

    public VoxelArray copy()
    {
        VoxelArray va = new VoxelArray(resolution);

        for (VoxelKey vk: voxels.keySet()) {
            va.voxels.put(vk, voxels.get(vk).copy());
        }

        return va;
    }

    class VoxelKey
    {
        public int[] xyz;

        public VoxelKey(double[] xyz_)
        {
            xyz = new int[3];
            xyz[0] = (int)(xyz_[0]/resolution);
            xyz[1] = (int)(xyz_[1]/resolution);
            xyz[2] = (int)(xyz_[2]/resolution);
        }

        public VoxelKey transform(Matrix rbt)
        {
            double[] xyz_xform = new double[] {xyz[0]+resolution/2,
                                               xyz[1]+resolution/2,
                                               xyz[2]+resolution/2};
            xyz_xform = LinAlg.transform(rbt, xyz_xform);

            return new VoxelKey(xyz_xform);
        }

        public int hashCode()
        {
            return Arrays.hashCode(xyz);
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof VoxelKey))
                return false;
            VoxelKey vk = (VoxelKey)o;
            return Arrays.equals(vk.xyz, xyz);
        }
    }
}
