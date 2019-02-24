using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace InverseKinematics
{
    class Program
    {
        static int arms = 3;
        static float[] armLength = { 50, 50, 50 };
        static float[] angleX = { 0, 0, 0 };
        static float[] angleY = { 0, 0, 0 };
        static float[] angleZ = { 0, 0, 0 };

        static void Main(string[] args)
        {
            //For 2D
            //DrawingWindow window = new DrawingWindow();

            //For 3D
            //_3D window = new _3D();
            //Application.Run(window);

            fkTest(true);

        }

        public static void fkTest(bool b)
        {
            float[] x = { 1f, 1f, 1f };
            float[] y = { 1f, 1f, 1f };
            float[] z = { 1f, 1f, 1f };
            float[] length = { 50, 50, 50 };
            float[] dest = { 70, 70, 70 };
            int[] Xmin = { 0, 0, 0 };
            int[] Xmax = { 360, 360, 360 };
            int[] Ymin = { 0, 0, 0 };
            int[] Ymax = { 360, 360, 360 };
            int[] Zmin = { 0, 0, 0 };
            int[] Zmax = { 360, 360, 360 };

            ForwardKinematics fk = new ForwardKinematics(x, y, z, length, Xmin, Xmax, Ymin, Ymax, Zmin, Zmax);

            if (b)
            {
                ikTest(fk, dest);
            }

            float[] ep = fk.getEndpoint();

            Console.WriteLine(ep[0] + " " + ep[1] + " " + ep[2]);
        }

        public static void ikTest(ForwardKinematics fk, float[] dest)
        {
            IK inv = new IK(fk, dest);
            float[][] finalAngles = inv.getAngles();
        }

        //static Quaternion rot(float theta, float phi, float alpha)
        //{
        //    Quaternion R = new Quaternion((float)Math.Cos(rad(theta / 2)), (float)Math.Sin(rad(theta / 2)), 0, 0);
        //    R *= new Quaternion((float)Math.Cos(rad(phi / 2)), 0, (float)Math.Sin(rad(phi / 2)), 0);
        //    R *= new Quaternion((float)Math.Cos(rad(alpha / 2)), 0, 0, (float)Math.Sin(rad(alpha / 2)));

        //    return R;
        //}

        //public static float pow(float i, float k)
        //{
        //    return (float)Math.Pow(i, k);
        //}

        //static double rad(float i)
        //{
        //    return i * Math.PI / 180;
        //}

        //static float sin(float d)
        //{
        //    return (float)(Math.Sin(rad(d)));
        //}

        //static float cos(float f)
        //{
        //    return (float) (Math.Cos(rad(f)));
        //}

        //static float getX(int i)
        //{
        //    float b = 0;
        //    for(int j = 0; j <= i; j++)
        //    {
        //        b += angleX[j];
        //    } return b;
        //}

        //static float getY(int i)
        //{
        //    float b = 0;
        //    for (int j = 0; j <= i; j++)
        //    {
        //        b += angleY[j];
        //    }
        //    return b;
        //}
        //static float getZ(int i)
        //{
        //    float b = 0;
        //    for (int j = 0; j <= i; j++)
        //    {
        //        b += angleZ[j];
        //    }
        //    return b;
        //}
    }
}
