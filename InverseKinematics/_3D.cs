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
        List<float>[] points;

        static int armCount = 3;
        float[] angleX = { 0, 0, 0 };
        float[] angleY = { 0, 0, 0 };
        float[] angleZ = { 0, 0, 0 };
        float[] armLength = { 50, 50, 50};
        float maxLength;

        Quaternion[] quatList;
        Quaternion[] XAxes = new Quaternion[armCount];
        Quaternion[] YAxes = new Quaternion[armCount];
        Quaternion[] ZAxes = new Quaternion[armCount];


        List<float> currentPosition = new List<float>();
        int[] finalPosition = { 70, 70, 70 };
        float cost;
        int counter = 0;

        //Testing optimal combinations
        float scaleFactor = 1500f;
        float del = 0.001f;


        public _3D() : base()
        {
            this.Width = 500;
            this.Height = 500;
            this.Paint += Draw;
            originX = this.Width / 2;
            originY = this.Height / 2;



            if (maxLength == 0)
            {
                foreach (int i in armLength)
                {
                    maxLength += i;
                }
            }

            int angleIncrement = 5;
            //Controls
            //Mouse click to move the arm to the location
            //Keys to move each arm individually
            this.MouseDown += (object sender, MouseEventArgs e) =>
            {
                Console.WriteLine("Click");
                Point p = target(new Point(e.Location.X, e.Location.Y));
                finalPosition[0] = 70;
                finalPosition[1] = 70;
                finalPosition[2] = 70;

                int l = 0;
                while (cost > 0.5)
                {
                    directMove();
                    l++;
                }
                Console.WriteLine("Iterations: " + l);
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
                if (e.KeyCode == Keys.P)
                {
                    calculatePartials();
                    this.Invalidate();
                }
            };
        }

        private void Draw(object sender, PaintEventArgs obj)
        {
            rotating(obj, true);
        }

        public void rotating()
        {
            rotating(null, false);
        }

        public void rotating(PaintEventArgs obj, bool draw)
        {
            for (int i = 0; i < angleX.Length; i++)
            {
                angleX[i] = angleX[i] % 360;
                angleY[i] = angleY[i] % 360;
                angleZ[i] = angleZ[i] % 360;
            }

            originX = this.Width / 2;
            originY = this.Height / 2;
            Point origin = new Point((int)originX, (int)originY);

            quatList = new Quaternion[armCount];

            Quaternion temp = new Quaternion(0, 1, 0, 0);
            Quaternion XAxis = new Quaternion(0, 1, 0, 0);
            Quaternion YAxis = new Quaternion(0, 0, 1, 0);
            Quaternion ZAxis = new Quaternion(0, 0, 0, 1);

            Quaternion R;
            Quaternion P;
            Quaternion Y;


            //Rotating
            for (int b = 0; b < armCount; b++)
            {
                float theta = angleX[b];
                float phi = angleY[b];
                float alpha = angleZ[b];


                R = new Quaternion((float)Math.Cos(rad(theta / 2)), (float)Math.Sin(rad(theta / 2)), 0, 0);
                Y = new Quaternion((float)Math.Cos(rad(phi / 2)), 0, (float)Math.Sin(rad(phi / 2)), 0);
                P = new Quaternion((float)Math.Cos(rad(alpha / 2)), 0, 0, (float)Math.Sin(rad(alpha / 2)));


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

                temp = (R * (Y * (P * temp * !P) * !Y) * !R);
                quatList[b] = temp.Normalized();

                XAxis = R * (Y * (P * XAxis * !P) * !Y) * !R; //x axis always equals temp so change later maybe
                YAxis = R * (Y * (P * YAxis * !P) * !Y) * !R;
                ZAxis = R * (Y * (P * ZAxis * !P) * !Y) * !R;

            }
            //Elongating
            for (int i = 0; i < armCount; i++)
            {
                quatList[i] *= armLength[i];
            }

            makePoints(obj, draw);
        }

        public void makePoints(PaintEventArgs obj, Boolean draw) //Possibly remove because points of every single joint is
                                                                 //only for drawing and not calculation
        {
            //Creating Points
            points = new List<float>[armCount];
            for (int b = 0; b < quatList.Length; b++)
            {
                points[b] = new List<float>();
                if (b == 0)
                {
                    points[b].Add(quatList[b].i);
                    points[b].Add(quatList[b].j);
                    points[b].Add(quatList[b].k);
                }
                else
                {
                    points[b].Add(points[b - 1][0] + quatList[b].i);
                    points[b].Add(points[b - 1][1] + quatList[b].j);
                    points[b].Add(points[b - 1][2] + quatList[b].k);
                }
            }
            if (draw)
            {
                model(obj);
            }
            currentPosition = points[points.Length - 1];
            cost = calculateCost();
        }

        //Draws the arm in the DrawingWindow 
        //Black Lines = Arm. Red Line = arm endpoint to desired endpoint
        public void model(PaintEventArgs obj)
        {
            //Drawing
            Graphics g = obj.Graphics;
            Point endP = new Point(0, 0);
            Point endP2 = new Point(0, 0);
            for (int a = 0; a < armCount; a++)
            {
                endP2 = new Point((int)points[a][0], (int)points[a][1]);
                g.DrawLine(Pens.Black, cartesian(endP), cartesian(endP2));
                endP = endP2;
            }

            g.DrawLine(Pens.Red, cartesian(endP), cartesian(new Point(finalPosition[0], finalPosition[1])));
            g.DrawEllipse(Pens.Red, originX - (maxLength), originY - (maxLength), 2*maxLength, 2*maxLength);
            //g.DrawEllipse()

            Console.WriteLine(points[points.Length - 1][0] + " " + points[points.Length - 1][1] + " " + points[points.Length - 1][2]);
            Console.WriteLine(cost);
            Console.WriteLine();
        }

        //Calulates dCost as if modeling the segments after the specified joint as a single Quaternion
        //Currently Unused
        public float dGamma(int joint)
        {
            //int delta = 1;
            Quaternion temp = new Quaternion(0, finalPosition[0] - points[joint - 1][0], finalPosition[1] - points[joint - 1][1], finalPosition[2] - points[joint - 1][2]);

            Quaternion a = XAxes[joint - 1] * (YAxes[joint - 1] * (ZAxes[joint - 1] * temp * !ZAxes[joint - 1]) * !YAxes[joint - 1]) * !(XAxes[joint - 1]);

            float x = points[joint - 1][0] + a.i;
            float y = points[joint - 1][1] + a.j;
            float z = points[joint - 1][2] + a.k;

            return (float)(Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2) + Math.Pow(z, 2)));
        }



        //Calculates and prints dCost for eact joint on the arm
        public void calculatePartials()
        {
            float delta = (float)(1);
            float c = cost;

            for (int i = 0; i < armCount; i++)
            {
                angleX[i] += delta;
                rotating(null, false);
                Console.WriteLine("Branch " + i + " dTheta: " + ((cost - c) / delta));
                angleX[i] -= delta;

                angleY[i] += delta;
                rotating(null, false);
                Console.WriteLine("Branch " + i + " dPhi: " + ((cost - c) / delta));
                angleY[i] -= delta;

                angleZ[i] += delta;
                rotating(null, false);
                Console.WriteLine("Branch " + i + " dBeta: " + ((cost - c) / delta));
                angleZ[i] -= delta;
                rotating(null, false);

                Console.WriteLine();
            }

        }


        //Returns the partial derivatives of every joint
        //Used in IK
        public float[,] returnPartials()
        {
            float delta = del;//(float)(0.0001);
            float c = cost;

            float[,] dAngles = new float[armCount, 3];
            float[][] angles = { angleX, angleY, angleZ };

            for (int i = 0; i < armCount; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    angles[j][i] += delta;
                    rotating();
                    dAngles[i, j] = ((c - cost));
                    angles[j][i] -= delta;
                }
            }
            return dAngles;
        }


        //Increments each joint in the right direction to reduce cost
        //Used in IK
        public void directMove()
        {
            float scaleFactor = this.scaleFactor;
            float[,] dAngles = returnPartials();
            float gMag = gradientMag(dAngles);

            for (int i = 0; i < armCount; i++)
            {
                angleX[i] += scaleFactor * dAngles[i, 0];
                angleY[i] += scaleFactor * dAngles[i, 1];
                angleZ[i] += scaleFactor * dAngles[i, 2];
            }

            rotating();
        }

        //Calculates the magnitude of the  gradient of decent
        //Currently unused
        public float gradientMag(float[,] dAngles)
        {
            float f = 0f;
            for (int i = 0; i < armCount; i++)
            {
                f += (float)(Math.Pow((double)(dAngles[i, 0]), 2));
                f += (float)(Math.Pow((double)(dAngles[i, 1]), 2));
                f += (float)(Math.Pow((double)(dAngles[i, 2]), 2));
            }
            return (float)(Math.Sqrt(((double)f)));
        }


        //Calculates current cost
        public float calculateCost()
        {
            return (float)Math.Sqrt(Math.Abs((Math.Pow(finalPosition[0] - currentPosition[0], 2) + Math.Pow(finalPosition[1] - currentPosition[1], 2) + Math.Pow(finalPosition[2] - currentPosition[2], 2))));
        }

        //Converts angle  inputed  to radians
        public double rad(float angle)
        {
            return (Math.PI / 180) * angle;
        }


        //Changes the DrawingWindow coordinates to cartesian coordinates centered in the middle
        //of the DrawingWindow
        public Point cartesian(Point p)
        {
            return cartesian(p.X, p.Y);
        }

        //Changes the DrawingWindow coordinates of the clicked IK target into cartesian coordinates
        public Point target(Point p)
        {
            return new Point(p.X - originX, originY - p.Y);
        }

        //cartesian(Point) but with and X and Y value instead of a point
        public Point cartesian(int x, int y)
        {
            return new Point(originX + x, originY - y);
        }

    }
}