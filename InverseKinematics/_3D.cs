using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace InverseKinematics
{
    class _3D : Form
    {

        int originX;
        int originY;
        Point origin;
        int angleIncrement = 10;
        static int armCount = 3;
        int[] angleX = { 0 , 0, 0};//new int[armCount];
        int[] angleY = { 0 , 0, 0};//new int[armCount];
        int[] angleZ = { 0 , 0, 0};//new int[armCount];
        int[] armLength = { 50, 50, 50};

        public _3D() : base()
        {
            this.Paint += Draw;
            this.MouseDown += (object sender, MouseEventArgs e) =>
            {
                this.Invalidate();
            };

            this.KeyDown += (object sender, KeyEventArgs e) =>
            {
                /////////////////////////////////BRANCH 0//////////////////////////////////
                if (e.KeyCode == Keys.W)
                {
                    angleX[0] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.Q)
                {
                    angleX[0] -= angleIncrement;
                    this.Invalidate();
                }
                if (e.KeyCode == Keys.S)
                {
                    angleY[0] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.A)
                {
                    angleY[0] -= angleIncrement;
                    this.Invalidate();
                }
                if (e.KeyCode == Keys.X)
                {
                    angleZ[0] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.Z)
                {
                    angleZ[0] -= angleIncrement;
                    this.Invalidate();
                }
                /////////////////////////////////BRANCH 1//////////////////////////////////
                if (e.KeyCode == Keys.R)
                {
                    angleX[1] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.E)
                {
                    angleX[1] -= angleIncrement;
                    this.Invalidate();
                }
                if (e.KeyCode == Keys.F)
                {
                    angleY[1] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.D)
                {
                    angleY[1] -= angleIncrement;
                    this.Invalidate();
                }
                if (e.KeyCode == Keys.V)
                {
                    angleZ[1] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.C)
                {
                    angleZ[1] -= angleIncrement;
                    this.Invalidate();
                }
                /////////////////////////////////BRANCH 2//////////////////////////////////
                if (e.KeyCode == Keys.Y)
                {
                    angleX[2] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.T)
                {
                    angleX[2] -= angleIncrement;
                    this.Invalidate();
                }
                if (e.KeyCode == Keys.H)
                {
                    angleY[2] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.G)
                {
                    angleY[2] -= angleIncrement;
                    this.Invalidate();
                }
                if (e.KeyCode == Keys.N)
                {
                    angleZ[2] += angleIncrement;
                    this.Invalidate();
                }
                else if (e.KeyCode == Keys.B)
                {
                    angleZ[2] -= angleIncrement;
                    this.Invalidate();
                }
            };

        }

        private void Draw(object sender, PaintEventArgs obj)
        {

            for (int i = 0; i < angleX.Length; i++)
            {
                if (angleX[i] % 360 == 0)
                {
                    angleX[i] = 0;
                }
                if (angleY[i] % 360 == 0)
                {
                    angleY[i] = 0;
                }
                if (angleZ[i] % 360 == 0)
                {
                    angleZ[i] = 0;
                }
            }

            originX = this.Width / 2;
            originY = this.Height / 2;
            origin = new Point((int)originX, (int)originY);

            Graphics g = obj.Graphics;

            List<float>[] points = new List<float>[armCount];
            //points[0] = new List<float>();
            //points[0].Add(0);
            //points[0].Add(0);
            //points[0].Add(0);

            Quaternion temp = new Quaternion(0, armLength[0], 0, 0);

            float theta = angleX[0];
            float phi = angleY[0];
            float alpha = angleZ[0];

            Quaternion R = new Quaternion((float)Math.Cos(rad(theta / 2)), (float)Math.Sin(rad(theta / 2)), 0, 0);
            Quaternion Y = new Quaternion((float)Math.Cos(rad(phi / 2)), 0, (float)Math.Sin(rad(phi / 2)), 0);
            Quaternion P = new Quaternion((float)Math.Cos(rad(alpha / 2)), 0, 0, (float)Math.Sin(rad(alpha / 2)));

            Quaternion XAxis = new Quaternion(0, 1, 0, 0);
            Quaternion YAxis = new Quaternion(0, 0, 1, 0);
            Quaternion ZAxis = new Quaternion(0, 0, 0, 1);

            for (int b = 0; b < armCount; b++)
            {
                points[b] = new List<float>();
                switch (b)
                {
                    case 0:
                        temp = new Quaternion(0, armLength[0], 0, 0);
                        break;
                    case 1:
                        temp = new Quaternion(0, points[b - 1][0], points[b - 1][1], points[b - 1][2]);
                        break;
                    default:
                        temp = new Quaternion(0, points[b - 1][0] - points[b - 2][0], points[b - 1][1] - points[b - 2][1],
                                                                                      points[b - 1][2] - points[b - 2][2]);
                        temp = temp.Normalized() * armLength[b];
                        break;
                }

                theta = angleX[b];
                phi = angleY[b];
                alpha = angleZ[b];

                double ro = Math.Sin(rad(theta / 2));
                R.r = (float)Math.Cos(rad(theta / 2));
                R.i = (float)(XAxis.i * ro); //Vx sin theta/2
                R.j = (float)(XAxis.j * ro);
                R.k = (float)(XAxis.k * ro);

                ro = Math.Sin(rad(phi / 2));
                Y.r = (float)Math.Cos(rad(phi / 2));
                Y.i = (float)(YAxis.i * ro);
                Y.j = (float)(YAxis.j * ro);
                Y.k = (float)(YAxis.k * ro);

                ro = Math.Sin(rad(alpha / 2));
                P.r = (float)Math.Cos(rad(alpha / 2));
                P.i = (float)(ZAxis.i * ro);
                P.j = (float)(ZAxis.j * ro);
                P.k = (float)(ZAxis.k * ro);

                temp = R * (Y * (P * temp * !P) * !Y) * !R;

                XAxis = R * (Y * (P * XAxis * !P) * !Y) * !R;
                YAxis = R * (Y * (P * YAxis * !P) * !Y) * !R;
                ZAxis = R * (Y * (P * ZAxis * !P) * !Y) * !R;

                switch (b)
                {
                    case 0:
                        points[b].Add(temp.i);
                        points[b].Add(temp.j);
                        points[b].Add(temp.k);
                        break;
                    default:
                        points[b].Add(points[b - 1][0] + temp.i);
                        points[b].Add(points[b - 1][1] + temp.j);
                        points[b].Add(points[b - 1][2] + temp.k);
                        break;
                }
                
            }


            //Drawing
            Point endP = new Point(0, 0);
            Point endP2 = new Point(0, 0);
            for (int a = 0; a < armCount; a++)
            {
                endP2 = new Point((int)points[a][0], (int)points[a][1]);
                g.DrawLine(Pens.Black, cartesian(endP), cartesian(endP2));
                endP = endP2;
            }
            Console.WriteLine(points[points.Length - 1][0] + " " + points[points.Length - 1][1]);
        }

        public int sumAngle(int i, int[] array)
        {
            int sum = 0;
            for (int j = 0; j <= i; j++)
            {
                sum += array[j];
            }
            return sum;
        }

        public double rad(float angle)
        {
            return (Math.PI / 180) * angle;
        }

        public Point cartesian(Point p)
        {
            return cartesian(p.X, p.Y);
        }

        public Point cartesian(int x, int y)
        {
            return new Point(originX + x, originY - y);
        }

    }
}
