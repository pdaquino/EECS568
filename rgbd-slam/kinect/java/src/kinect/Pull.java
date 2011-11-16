package kinect;

import april.util.*;

class Pull
{
    static public void main(String[] args)
    {
        double seconds = 2;

        Kinect k = new Kinect();
        k.init();
        k.start();
        int f_cnt = 0;
        Tic tic = new Tic();
        while (true) {
            Kinect.Frame f = k.getFrame();
            if (f != null)
                f_cnt++;
            if (tic.toc() > seconds)
                break;
        }
        k.stop();
        System.out.printf("Expected frames: %d\n", f_cnt);
        k.printCount();

        System.out.printf("Shutting down kinect\n");
        k.close();
    }
}
