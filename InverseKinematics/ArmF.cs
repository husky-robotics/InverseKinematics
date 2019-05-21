using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InverseKinematics
{
    public class ArmF
    {
        public ArmPartF[] parts;
        public float[] Zcur;
        public TDPoint[] points;
        public double cost;
        public float baser;

        public ArmF(float[] z, float br, ArmPartF[] segs)
        {
            Zcur = z;
            points = new TDPoint[z.Length + 1];
            baser = br;
            parts = segs;
            //findPoints();
        }

        public void converge(TDPoint target) {
            findPoints();
            TDPoint tShadow = new TDPoint(target.x, 0, target.z);
            TDPoint aShadow = new TDPoint(points[points.Length - 1].x, 0, points[points.Length - 1].z);

            //decide sign of baseRotation
            double baseRotation = aShadow.angleBetween(tShadow);
            Console.WriteLine("Base Rotation: " + baseRotation);

            double t = Math.Atan(target.y / (Math.Sqrt(target.x * target.x + target.z * target.z)));
            TDPoint tar = new TDPoint((float)(target.Length * Math.Cos(t)), target.y, 0);

            float zTot = 0;
            points[0] = new TDPoint(0, 0, 0);

            for (int i = 1; i < points.Length; i++) {
                zTot += Zcur[i - 1];
                points[i] = new TDPoint((float)(parts[i - 1].length * Math.Cos(zTot)),
                                        (float)(parts[i - 1].length * Math.Sin(zTot)),
                                         0) + points[i - 1];
            }
            cost = calcCost(target);

            int count = 1;

            while (cost > 0.0001) {
                ////////////////////////////////////////////Moving Joints////////////////////////////////////////////
                for (int i = points.Length - 2; i >= 0; i--) {
                    if (i == points.Length - 2) {
                        points[i + 1] = tar;
                    }
                    points[i] = points[i + 1] + ((points[i] - points[i + 1]).Normalized() * parts[i].length);
                }

                points[0].set(0, 0, 0);
                for (int i = 1; i < points.Length; i++) {
                    points[i] = points[i - 1] + ((points[i] - points[i - 1]).Normalized() * parts[i - 1].length);
                }

                /////////////////////////////////////////Correcting Angles////////////////////////////////////////////
                for (int i = points.Length - 2; i > 0; i--) {
                    double ang = (points[i] | points[i - 1]).angleRelativeTo(points[i + 1] | points[i]);
                    if (ang < parts[i].Zmin || ang > parts[i].Zmax) {
                        if (ang < parts[i].Zmin) {
                            Zcur[i] = parts[i].Zmin;
                        }
                        else {
                            Zcur[i] = parts[i].Zmax;
                        }
                    }
                    else {
                        Zcur[i] = (float)ang;
                    }
                }
                double angle = (new TDPoint(1, 0, 0)).angleRelativeTo(points[1]);
                if (angle < parts[0].Zmin) {
                    Zcur[0] = parts[0].Zmin;
                }
                else if (angle > parts[0].Zmax) {
                    Zcur[0] = parts[0].Zmax;
                }
                else {
                    Zcur[0] = (float)angle;
                }

                //////////////////////////////////////Recalculating Points/////////////////////////////////////////
                //{
                    float curZ = 0;
                    points[0].set(0, 0, 0);
                    for (int i = 1; i < this.points.Length; i++) {
                    curZ = (float)((curZ + this.Zcur[i - 1]));// % (2 * Math.PI));
                        points[i] = points[i - 1] + (new TDPoint((float)(parts[i - 1].length * Math.Cos(curZ)), (float)(parts[i - 1].length * Math.Sin(curZ)), 0));
                    }
                //}
                cost = calcCost(tar);
                count++;
            }


            Console.WriteLine(baseRotation);
            printPoints();
            Console.WriteLine(count);
            Console.WriteLine();
        }


        public void printPoints()
        {
            for (int i = 0; i < points.Length; i++)
            {
                points[i].print();
            }
        }

        public void printAngles()
        {
            for (int i = 0; i < Zcur.Length; i++)
            {
                Console.WriteLine(Zcur[i] * 180 / Math.PI);
            }
        }

        //Figure out with our changed axes
        public void findPoints() {
            float z = 0;
            points[0] = new TDPoint(0, 0, 0);
            for (int i = 0; i < Zcur.Length; i++) {
                z += Zcur[i];
                points[i + 1] = points[i] + new TDPoint((float)(parts[i].length * Math.Cos(baser) * Math.Cos(z)),
                                                        (float)(parts[i].length * Math.Sin(z)),
                                                        (float)(parts[i].length * Math.Cos(z) * Math.Sin(baser)));
            }
            Console.WriteLine();
        }

        public double calcCost(TDPoint target) {
            return Math.Sqrt(Math.Pow(points[points.Length - 1].x - target.x, 2) + Math.Pow(points[points.Length - 1].y - target.y, 2) + Math.Pow(points[points.Length - 1].z - target.z, 2));
        }

    }

    public class ArmPartF
    {
        public float Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;
        public float length;

        public ArmPartF(float length, float Xmin, float Xmax, float Ymin, float Ymax, float Zmin, float Zmax)
        {
            this.length = length;
            this.Xmin = Xmin;
            this.Xmax = Xmax;
            this.Ymin = Ymin;
            this.Ymax = Ymax;
            this.Zmin = Zmin;
            this.Zmax = Zmax;
        }
    }
}
