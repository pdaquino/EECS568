package team;

public class LandmarkPose implements Node
{
    private double[] position = {0, 0};
    int index;

    public LandmarkPose(int index) {
        this.index = index;
    }

    public double[] getPosition() {
        return this.position;
    }

    public void setPosition(double[] position) {
        this.position = position;
    }

    public int getIndex() {
        return index;
    }

    public int getNumDimensions() {
        return 2;
    }
}
