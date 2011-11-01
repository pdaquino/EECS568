package april.vis;

import java.util.*;

/** It is legal to append new data to a VisData, but it is not allowed
 * to change or remove data.
 **/
public class VisData
{
    boolean hasDouble;
    int maxDimension;

    ArrayList<Object> points = new ArrayList<Object>();

    public void add(Object ... args)
    {
        for (int i = 0; i < args.length; i++) {

        }
    }
}
