/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package kinect;

/**
 *
 * @author jrpeterson
 */
public class Pixel {
    public int x;
    public int y;
    
    public Pixel(int x, int y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public int hashCode() {
        int hash = 7;
        hash = 59 * hash + this.x;
        hash = 59 * hash + this.y;
        return hash;
    }
    
    @Override
    public boolean equals(Object that) {
        if (that instanceof Pixel) {
            return equals((Pixel) that);
        } else {
            return false;
        }
    }
    
    public boolean equals(Pixel that) {
        return ((x == that.x) && (y == that.y));
    }
    
    
}
