using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ConsoleApplicationTest1
{
    class _3D : Form
    {

        int originX;
        int originY;
        Point origin;
        int angleIncrement = 10;
        static int armCount = 2;
        int[] angleX = new int[armCount];
        int[] angleY = new int[armCount];
        int[] angleZ = new int[armCount];
        int armLength = 50;

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

            float theta = 0;
            float phi = 0;
            float alpha = (float)Math.PI / 2;

            //Quaternion R = new Quaternion((float)Math.Cos(theta / 2), (float)Math.Sin(theta / 2), 0, 0);
            //Quaternion P = new Quaternion((float)Math.Cos(phi / 2), 0, (float)Math.Sin(phi / 2), 0);
            //Quaternion Y = new Quaternion((float)Math.Cos(alpha / 2), 0, 0, (float)Math.Sin(alpha / 2));
            //Quaternion Rconj = !R;
            //Quaternion Pconj = !P;
            //Quaternion Yconj = !Y;
            Graphics g = obj.Graphics;

            List<Quaternion> quatList = new List<Quaternion>();

            for (int i = 0; i < angleX.Length; i++)
            {
                quatList.Add(new Quaternion(0, armLength, 0, 0));
            }

            Point endP = new Point(0, 0);
            Point endP2 = new Point(0, 0);

            for (int b = 0; b < quatList.Count; b++)
            {
                theta = getAngle(b, angleX);
                phi = getAngle(b, angleY);
                alpha = getAngle(b, angleZ);

                Quaternion R = new Quaternion((float)Math.Cos(rad(theta / 2)), (float)Math.Sin(rad(theta / 2)), 0, 0);
                Quaternion Y = new Quaternion((float)Math.Cos(rad(phi / 2)), 0, (float)Math.Sin(rad(phi / 2)), 0);
                Quaternion P = new Quaternion((float)Math.Cos(rad(alpha / 2)), 0, 0, (float)Math.Sin(rad(alpha / 2)));
                Quaternion Rconj = !R;
                Quaternion Yconj = !Y;
                Quaternion Pconj = !P;

                Quaternion temp = quatList[b];
                quatList[b] = R * (Y * (P * temp * Pconj) * Yconj) * Rconj;
            }


            for (int a = 0; a < quatList.Count; a++)
            {
                if (a == 0)
                {
                    endP = origin;
                }
                endP2 = new Point((int)((Math.Round(endP.X + (quatList[a].i)))),
                                  (int)((Math.Round(endP.Y - (quatList[a].j)))));
                
                g.DrawLine(Pens.Black, endP, endP2);
                endP = endP2;
            }
            Console.WriteLine(angleX[0] + " " + angleY[0] + " " + angleZ[0]);
            Console.WriteLine(cartesian(endP2));
        }

        public int getZ(List<Quaternion> list)
        {
            float sum = 0;

            for(int i = 0; i < list.Count; i++)
            {
                sum += (list[i].Length() * angleZ[i]);
            }

            return (int)sum;

        }

        public int getAngle(int i, int[] array)
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
            return new Point(-(originX - p.X), originY - p.Y);
        }

        public Point cartesian(int x, int y)
        {
            return new Point(originX + x, originY - y);
        }

    }
}
