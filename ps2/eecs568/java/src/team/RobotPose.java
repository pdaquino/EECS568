package team;

/**
 *
 * @author pdaquino
 */
public class RobotPose implements Node {

    private double[] position = {0, 0 , 0};
    int index;
    public RobotPose(int index) {
        this.index = index;
    }
    
    @Override
    public double[] getPosition() {
        return this.position;
    }

    @Override
    public void setPosition(double[] position) {
        this.position = position;
    }

    @Override
    public int getIndex() {
        return index;
    }

    @Override
    public int getNumDimensions() {
        return 3;
    }
    
}
