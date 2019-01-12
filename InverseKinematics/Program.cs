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
        static float[] armLength = { 50, 50, 50, 50 };
        static float[] angleX = { 0, 0, 0, 0 };
        static float[] angleY = { 0, 0, 0, 0 };
        static float[] angleZ = { 90, 90, 0, 0 };

        static void Main(string[] args)
        {
            //For 2D
            //DrawingWindow window = new DrawingWindow();

            //For 3D
            _3D window = new _3D();

            Application.Run(window);
            //            FKDualQuat();
            

        }


        static void IKQuat(float[] the, float[] phi, float[] bet,float[] lengths, float[] end)
        {
            float xc = 0;
            float yc = 0;
            float zc = 90;


            float dTheta = 0;
            float dPhi = 0;
            float dBeta = 0;

            float[,] der = new float[3, the.Length];

            List<float>[] deri = new List<float>[the.Length];

            for(int i = 0; i < der.Length; i++)
            {
                Quaternion qRot = rot(the[i], phi[i], bet[i]);
                Quaternion dT = rot(the[i] + dTheta, phi[i], bet[i]);
                Quaternion dP = rot(the[i], phi[i], bet[i]);
                Quaternion dB = rot(the[i], phi[i], bet[i]);

                Quaternion dadt = (dT - qRot) / dTheta;
                Quaternion dadp = (dT - qRot) / dPhi;
                Quaternion dadb = (dT - qRot) / dBeta;

                //deri[i].Add(dadt.dGam(end));
                //deri[i].Add(dadp.dGam(end));
                //deri[i].Add(dadb.dGam(end));

            }

            for (int i = 0; i < deri.Length; i++)
            {
                the[i] += deri[i][0];
                phi[i] += deri[i][1];
                bet[i] += deri[i][2];
            }

            if (pow(end[0] - xc, 2) + pow(end[1] - yc, 2) + pow(end[2] - zc, 2) > 1)
            {
                IKQuat(the, phi, bet, lengths, end);
            }

        }







        //static void FKDualQuat()
        //{
            


        //    DualQuaternion[] quatList = new DualQuaternion[arms];
        //    DualQuaternion endDualQuat = new DualQuaternion(new Quaternion(0,0,0,0), new Quaternion(0,0,0,0));

        //    for (int i = 0; i < arms; i++)
        //    {
        //        //Quaternion R = new Quaternion((float)Math.Cos(rad(angleX[i] / 2)), (float)Math.Sin(rad(angleX[i] / 2)), 0, 0);
        //        //Quaternion Y = new Quaternion((float)Math.Cos(rad(angleY[i] / 2)), 0, (float)Math.Sin(rad(angleY[i] / 2)), 0);
        //        //Quaternion P = new Quaternion((float)Math.Cos(rad(angleZ[i] / 2)), 0, 0, (float)Math.Sin(rad(angleZ[i] / 2)));

        //        Quaternion R = new Quaternion((float)cos((angleX[i] / 2)), (float)sin((angleX[i] / 2)), 0, 0);
        //        Quaternion Y = new Quaternion((float)cos((angleY[i] / 2)), 0, (float)sin((angleY[i] / 2)), 0);
        //        Quaternion P = new Quaternion((float)cos((angleZ[i] / 2)), 0, 0, (float)sin((angleZ[i] / 2)));

        //        Quaternion qRot = //new Quaternion(1, 0, 0, 0);
        //            R * Y * P;

        //        DualQuaternion qDRotation = new DualQuaternion(qRot, new Quaternion(0, 0, 0, 0));

        //        DualQuaternion qTrans = new DualQuaternion(new Quaternion(1, 0, 0, 0), (new Quaternion(0, armLength[i] / 2, 0, 0)));
        //        //DualQuaternion qTrans = new DualQuaternion(qRot, (new Quaternion(0, armLength[i] * cos(getAngle(i, angleZ)) / 2, armLength[i] * sin(getAngle(i, angleZ)) / 2,  0)));

        //        quatList[i] = qTrans * qDRotation;

        //        //quatList[i].print();


        //        //quatList[i] = new DualQuaternion(qRot, (new Quaternion(0, (armLength[i]) / 2, 0, 0)) * qRot);

        //        Console.WriteLine("Arm Segment " + i);
        //        quatList[i].Re.zero();
        //        quatList[i].Re.print();
        //        quatList[i].Du.zero();
        //        quatList[i].Du.print();

        //        //if (i == 0)
        //        //{
        //        //    endDualQuat = quatList[0];
        //        //}
        //        //else
        //        //{
        //        //    endDualQuat = endDualQuat * quatList[i];
        //        //}
        //        Console.WriteLine();
        //    }

        //    endDualQuat = quatList[0];

        //    for(int i = 1; i < arms; i++)
        //    {
        //        endDualQuat = endDualQuat * quatList[i];
        //    }

        //    Console.WriteLine("endDualQuat");
        //    endDualQuat.print();
        //    Console.WriteLine();

        //    //(endDualQuat.Re * !endDualQuat.Re).print();

        
        //    Quaternion endQuat = (endDualQuat.Re) * (2 *(endDualQuat.Du) * (!endDualQuat.Re)) * (!endDualQuat.Re);

        //    //endDualQuat.Re.print();
        //    //endDualQuat.Du.print();

        //    //(!endDualQuat.Re).print();

        //    //(((endDualQuat.Du)) * (!endDualQuat.Re)).print();

        //    Console.WriteLine("Final Position");
        //    endQuat.zero();
        //    endQuat.print();

        //    //temp.print();

        //    //endQuat.print();

        //    //Quaternion q1 = new Quaternion((float)(Math.Sqrt(2) / 2), 0, 0, (float)( Math.Sqrt(2) / 2));
        //    //Quaternion q2 = new Quaternion(0, (float)(25 * Math.Sqrt(2) / 2), -25 * (float)(Math.Sqrt(2)) / 2, 0);
        //    //((q1 * q2) + (q2 * q1)).print();


        //}


        static Quaternion rot(float theta, float phi, float alpha)
        {
            Quaternion R = new Quaternion((float)Math.Cos(rad(theta / 2)), (float)Math.Sin(rad(theta / 2)), 0, 0);
            R *= new Quaternion((float)Math.Cos(rad(phi / 2)), 0, (float)Math.Sin(rad(phi / 2)), 0);
            R *= new Quaternion((float)Math.Cos(rad(alpha / 2)), 0, 0, (float)Math.Sin(rad(alpha / 2)));

            return R;
        }

        public static float pow(float i, float k)
        {
            return (float)Math.Pow(i, k);
        }

        static double rad(float i)
        {
            return i * Math.PI / 180;
        }

        static float sin(float d)
        {
            return (float)(Math.Sin(rad(d)));
        }

        static float cos(float f)
        {
            return (float) (Math.Cos(rad(f)));
        }

        static float getX(int i)
        {
            float b = 0;
            for(int j = 0; j <= i; j++)
            {
                b += angleX[j];
            } return b;
        }

        static float getY(int i)
        {
            float b = 0;
            for (int j = 0; j <= i; j++)
            {
                b += angleY[j];
            }
            return b;
        }
        static float getZ(int i)
        {
            float b = 0;
            for (int j = 0; j <= i; j++)
            {
                b += angleZ[j];
            }
            return b;
        }
    }
}
