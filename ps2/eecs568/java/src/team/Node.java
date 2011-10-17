package team;

/**
 *
 * @author pdaquino
 */
public interface Node {
    double[] getPosition();
    void setPosition(double[] position);
    // gets the Node's index in the state vector
    int getIndex();
    int getNumDimensions();
}
