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

            //fkTest(true);

            FABRIKTest();
        }

        public static void alignmentTest()
        {
            TDPoint t = new TDPoint(10, 10, 0);
        }

        public static void fkTest(bool b)
        {
            float[] x = { 0f, 0f, 0f };
            float[] y = { 0f, 0f, 0f };
            float[] z = { 0f, 0f, 0f };
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

        public static void FABRIKTest()
        {
            TDPoint target = new TDPoint(50f, 50f, 50f);
            float r = (float)(Math.PI / 180);

            ArmPartQ a1 = new ArmPartQ(0, 0, 10, 0,
                                       0, 0, 0, 0, 90 * r, 90 * r);
            ArmPartQ a2 = new ArmPartQ(0, 50, 0, 0,
                                       0, 0, 0, 0, 0 * r , 360 * r);
            ArmPartQ a3 = new ArmPartQ(0, 50, 0, 0,
                                       0, 0, 0, 0, 0 * r , 360 * r);
            ArmPartQ a4 = new ArmPartQ(0, 50, 0, 0,
                                       0, 0, 0, 0, 0 * r, 180 * r);

            ArmPartQ[] a10 = {a1, a2, a3, a4};

            ArmQ arm1 = new ArmQ(a10);
            FABRIK f = new FABRIK(arm1);
            arm1.converge(target);
            //f.converge2D(target);
        }

        public static double rad(float angle)
        {
            return (Math.PI / 180) * angle;
        }
    }
}
