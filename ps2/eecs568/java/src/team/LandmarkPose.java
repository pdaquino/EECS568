package team;

import april.jmat.LinAlg;

public class LandmarkPose implements Node {

    private double[] position = {0, 0}; // (x,y)
    int index;
    int id = -1;

    public LandmarkPose(int index) {
        this.index = index;
    }

    public LandmarkPose(int index, RobotPose latestPose, double r, double theta) {
        this.index = index;
        double[] rel_xy = new double[]{r * Math.cos(theta),
            r * Math.sin(theta)};
        this.position = LinAlg.transform(latestPose.getPosition(), rel_xy);
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

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }
}
