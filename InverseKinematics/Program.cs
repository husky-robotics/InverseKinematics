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
            TDPoint target = new TDPoint(70f, 60f, 80f);
            float r = (float)(Math.PI / 180);

            ArmPartQ a = new ArmPartQ(50, 0, 0, 0, 0, 0 * r, 180 * r);
            ArmPartQ b = new ArmPartQ(50, 0, 0, 0, 0, 0 * r, 180 * r);
            ArmPartQ c = new ArmPartQ(50, 0, 0, 0, 0, 0 * r, 180 * r);

            ArmPartQ[] asdf = { a, b, c };
            float[] Zs = { 0f, 0f, 0f };

            ArmQ arm1 = new ArmQ(Zs, 0 * r, asdf);
            arm1.converge(target);
            arm1.printAngles();
        }

        public static double rad(float angle)
        {
            return (Math.PI / 180) * angle;
        }
    }
}
