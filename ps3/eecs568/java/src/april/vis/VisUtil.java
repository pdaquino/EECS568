package april.vis;

import java.util.concurrent.atomic.*;

import april.jmat.*;

import java.awt.*;
import java.awt.image.*;
import java.util.concurrent.atomic.*;

public class VisUtil
{
    static AtomicLong nextId = new AtomicLong(1);

    public static long allocateID()
    {
        return nextId.getAndIncrement();
    }

    public static BufferedImage coerceImage(BufferedImage input, int type)
    {
        if (input.getType() == type)
            return input;

        // coerce texture format to correct type.
        BufferedImage im = new BufferedImage(input.getWidth(),
                                             input.getHeight(),
                                             type);
        Graphics g = im.getGraphics();
        g.drawImage(input, 0, 0, input.getWidth(), input.getHeight(), null);
        g.dispose();

/*        if (!warnedSlowConversion) {
            System.out.println("VisTexture: Slow image type conversion for type "+input.getType());
            warnedSlowConversion = true;
        }
*/
        return im;
    }

    public static double[][] gluPerspective(double fovy_degrees, double aspect, double znear, double zfar)
    {
        double M[][] = new double[4][4];

        double f = 1.0 / Math.tan(Math.toRadians(fovy_degrees)/2);

        M[0][0] = f/aspect;
        M[1][1] = f;
        M[2][2] = (zfar+znear)/(znear-zfar);
        M[2][3] = 2*zfar*znear / (znear-zfar);
        M[3][2] = -1;

        return M;
    }

    public static double[][] glOrtho(double left, double right, double bottom, double top, double znear, double zfar)
    {
        double M[][] = new double[4][4];

        M[0][0] = 2 / (right - left);
        M[0][3] = -(right+left)/(right-left);
        M[1][1] = 2 / (top-bottom);
        M[1][3] = -(top+bottom)/(top-bottom);
        M[2][2] = -2 / (zfar - znear);
        M[2][3] = -(zfar+znear)/(zfar-znear);
        M[3][3] = 1;

        return M;
    }

    public static double[][] lookAt(double eye[], double c[], double up[])
    {
        up = LinAlg.normalize(up);
        double f[] = LinAlg.normalize(LinAlg.subtract(c, eye));

        double s[] = LinAlg.crossProduct(f, up);
        double u[] = LinAlg.crossProduct(s, f);

        double M[][] = new double[4][4];
        M[0][0] = s[0];
        M[0][1] = s[1];
        M[0][2] = s[2];
        M[1][0] = u[0];
        M[1][1] = u[1];
        M[1][2] = u[2];
        M[2][0] = -f[0];
        M[2][1] = -f[1];
        M[2][2] = -f[2];
        M[3][3] = 1;

        double T[][] = new double[4][4];
        T[0][3] = -eye[0];
        T[1][3] = -eye[1];
        T[2][3] = -eye[2];
        T[0][0] = 1;
        T[1][1] = 1;
        T[2][2] = 1;
        T[3][3] = 1;
        return LinAlg.matrixAB(M, T);
    }

    /** uses winy=0 at the bottom (opengl) convention **/
    public static double[] project(double xyz[], double M[][], double P[][], int viewport[])
    {
        double xyzh[] = new double[] { xyz[0], xyz[1], xyz[2], 1 };
        double p[] = LinAlg.matrixAB(LinAlg.matrixAB(P, M), xyzh);

        p[0] = p[0] / p[3];
        p[1] = p[1] / p[3];
        p[2] = p[2] / p[3];

        return new double[] { viewport[0] + viewport[2]*(p[0]+1)/2.0,
                              viewport[1] + viewport[3]*(p[1]+1)/2.0,
                              (viewport[2] + 1)/2.0 };
    }

    /** uses winy=0 at the bottom (opengl) convention **/
    public static double[] unProject(double winxyz[], double M[][], double P[][], int viewport[])
    {
        double invPM[][] = LinAlg.inverse(LinAlg.matrixAB(P, M));

        double v[] = new double[] { 2*(winxyz[0]-viewport[0]) / viewport[2] - 1,
                                    2*(winxyz[1]-viewport[1]) / viewport[3] - 1,
                                    2*winxyz[2] - 1,
                                    1 };

        double objxyz[] = LinAlg.matrixAB(invPM, v);
        objxyz[0] /= objxyz[3];
        objxyz[1] /= objxyz[3];
        objxyz[2] /= objxyz[3];

        return objxyz;
    }

    // vertically flip image
    public static void flipImage(int stride, int height, int b[])
    {
        int tmp[] = new int[stride];

        for (int row = 0; row < (height-1)/2; row++) {

            int rowa = row;
            int rowb = height-1 - rowa;

            // swap rowa and rowb

            // tmp <-- rowa
            System.arraycopy(b, rowa*stride, tmp, 0, stride);

            // rowa <-- rowb
            System.arraycopy(b, rowb*stride, b, rowa*stride, stride);

            // rowb <-- tmp
            System.arraycopy(tmp, 0, b, rowb*stride, stride);
        }
    }

}
