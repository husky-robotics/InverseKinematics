using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InverseKinematics
{
    class ForwardKinematics
    {
        public float[] angleX;
        public float[] angleY;
        public float[] angleZ;
        public float[] armLength;
        public int armCount;

        public ArmPartQ[] quatList;
        Quaternion[] XAxes;
        Quaternion[] YAxes;
        Quaternion[] ZAxes;

        int[] Xmin;
        int[] Xmax;
        int[] Ymin;
        int[] Ymax;
        int[] Zmin;
        int[] Zmax;

        public ForwardKinematics(float[] angleX, float[] angleY, float[] angleZ, float[] armLength,
                                   int[] minX, int[] maxX, int[] minY, int[] maxY, int[] minZ, int[] maxZ)
        {
            armCount = angleX.Length;
            this.angleX = angleX;
            this.angleY = angleY;
            this.angleZ = angleZ;
            this.armLength = armLength;
            quatList = new ArmPartQ[armCount];
            XAxes = new Quaternion[armCount];
            YAxes = new Quaternion[armCount];
            ZAxes = new Quaternion[armCount];
            Xmin = minX;
            Xmax = maxX;
            Ymin = minY;
            Ymax = maxY;
            Zmin = minZ;
            Zmax = maxZ;
            solve();
        }


        public void solve(int armNum)
        {
            Quaternion arm;
            Quaternion XAxis;
            Quaternion YAxis;
            Quaternion ZAxis;

            if (armNum == 0)
            {
                arm = new Quaternion(0, 1, 0, 0);
                XAxis = new Quaternion(0, 1, 0, 0);
                YAxis = new Quaternion(0, 0, 1, 0);
                ZAxis = new Quaternion(0, 0, 0, 1);
            }
            else
            {
                arm = quatList[armNum].arm.Normalized();
                XAxis = XAxes[armNum];
                YAxis = YAxes[armNum];
                ZAxis = ZAxes[armNum];
            }

            //Quaternion R;
            //Quaternion P;
            //Quaternion Y;

            //Rotating
            for (int b = armNum; b < angleX.Length; b++)
            {

                float theta, phi, alpha;

                //checks if angle is fixed or not. A negative angle means the angle of the joint
                //is fixed at that angle (-30 means fixed at 30)
                {
                    if (angleX[b] < 0)
                    {
                        theta = -angleX[b];
                    }
                    else
                    {
                        theta = angleX[b];
                    }
                    if (angleY[b] < 0)
                    {
                        phi = -angleY[b];
                    }
                    else
                    {
                        phi = angleY[b];
                    }
                    if (angleZ[b] < 0)
                    {
                        alpha = -angleZ[b];
                    }
                    else
                    {
                        alpha = angleZ[b];
                    }
                }

                double ro = Math.Sin(rad(theta / 2));
                Quaternion R = new Quaternion((float)Math.Cos(rad(theta / 2)), (float)(XAxis.i * ro), (float)(XAxis.j * ro), (float)(XAxis.k * ro));

                ro = Math.Sin(rad(phi / 2));
                Quaternion Y = new Quaternion((float)Math.Cos(rad(phi / 2)), (float)(YAxis.i * ro), (float)(YAxis.j * ro), (float)(YAxis.k * ro));

                ro = Math.Sin(rad(alpha / 2));
                Quaternion P = new Quaternion((float)Math.Cos(rad(alpha / 2)), (float)(ZAxis.i * ro), (float)(ZAxis.j * ro), (float)(ZAxis.k * ro));

                arm = (R * (Y * (P * arm * !P) * !Y) * !R);
                quatList[b] = new ArmPartQ(arm.r, arm.i, arm.j, arm.k, Xmin[b], Xmax[b], Ymin[b], Ymax[b], Zmin[b], Zmax[b]);

                XAxes[b] = XAxis;
                YAxes[b] = YAxis;
                ZAxes[b] = ZAxis;

                XAxis = R * (Y * (P * XAxis * !P) * !Y) * !R; //x axis always equals temp so change later maybe
                YAxis = R * (Y * (P * YAxis * !P) * !Y) * !R;
                ZAxis = R * (Y * (P * ZAxis * !P) * !Y) * !R;
            }

            //Elongating
            for (int i = armNum; i < armLength.Length; i++)
            {
                quatList[i].arm *= armLength[i];
            }
        }

        public void solve()
        {
            solve(0);
        }

        public float[] getEndpoint()
        {
            float[] ep = new float[3];
            //foreach(Quaternion q in quatList)
            for(int i = 0; i < quatList.Length; i++)
            {
                ep[0] += quatList[i].arm.i;
                ep[1] += quatList[i].arm.j;
                ep[2] += quatList[i].arm.k;
            }
            return ep;
        }


        //unused
        public void incrementAngle(int armNumber, int dimension, float d)
        {
            switch (dimension)
            {
                case 0:
                    angleX[armNumber] += d;
                    //solve(armNumber);
                    solve();
                    angleX[armNumber] -= d;
                    break;
                case 1:
                    angleY[armNumber] += d;
                    //solve(armNumber);
                    solve();
                    angleY[armNumber] -= d;
                    break;
                case 2:
                    angleZ[armNumber] += d;
                    //solve(armNumber);
                    solve();
                    angleZ[armNumber] -= d;
                    break;
            }

        }

        //Converts angle  inputed  to radians
        public double rad(float angle)
        {
            return (Math.PI / 180) * angle;
        }
    }
}
