using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InverseKinematics
{
    public class ArmPartQ
    {
        public Quaternion arm;
        public float Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;
        //public bool Xfix, Zfix;

        public ArmPartQ(float r, float i, float j, float k, float Xmin, float Xmax, float Ymin, float Ymax, float Zmin, float Zmax)
        {
            arm = new Quaternion(r, i, j, k);
            this.Xmin = Xmin;
            this.Xmax = Xmax;
            this.Ymin = Ymin;
            this.Ymax = Ymax;
            this.Zmin = Zmin;
            this.Zmax = Zmax;
        }
    }

    public class ArmQ
    {
        public ArmPartQ[] parts;
        public float[] Zcur;
        public TDPoint[] points;
        public double cost;

        public ArmQ(ArmPartQ[] segs)
        {
            parts = segs;
            Zcur = new float[parts.Length];
            points = new TDPoint[parts.Length + 1];
            updatePoints();
        }

        public void converge(TDPoint target)
        {
            TDPoint tShadow = new TDPoint(target.x, 0, target.z);
            TDPoint aShadow = new TDPoint(points[points.Length - 1].x, 0, points[points.Length - 1].z);

            //decide sign of baseRotation
            double baseRotation = aShadow.angleBetween(tShadow);
            Console.WriteLine(baseRotation);

            double t = Math.Atan(target.y / (Math.Sqrt(target.x * target.x + target.z * target.z)));
            TDPoint tar = new TDPoint((float)(target.Length * Math.Cos(t)), target.y, 0);

            float zTot = 0;
            double distance;
            points[0] = new TDPoint(0, 0, 0);
            for (int i = 1; i < points.Length; i++) {
                zTot += Zcur[i - 1];
                distance = points[i].Length;
                points[i] = new TDPoint((float)(distance * Math.Cos(zTot)), (float)(distance * Math.Sin(zTot)), 0) + points[i - 1];
            }

            cost = calcCost(target);
            ////////////////////////////////////////////Moving Joints////////////////////////////////////////////
            while (cost > 0.001) {
                for (int i = points.Length - 2; i >= 0; i--){
                    if (i == points.Length - 2) {
                        points[i + 1] = tar;
                    }
                    points[i] = points[i + 1] + ((points[i] - points[i + 1]).Normalized() * parts[i].arm.Length);
                }

                points[0].set(0, 0, 0);
                for (int i = 1; i < points.Length; i++) {
                    parts[i - 1].arm = ((points[i] - points[i - 1]).Normalized() * parts[i - 1].arm.Length);
                    points[i] = points[i - 1] + parts[i - 1].arm;
                }

                //angle constraints
                double[] changes = new double[points.Length - 1];

                for (int i = points.Length - 2; i > 0; i--) {
                    //TDPoint pre = points[i] | points[i - 1];
                    //TDPoint post = points[i + 1] | points[i];

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

                //recalculate points
                {
                    float Zcur = 0;
                    points[0].set(0, 0, 0);
                    for (int i = 1; i < this.Zcur.Length; i++)
                    {
                        Zcur += this.Zcur[i - 1];
                        points[i] = points[i - 1] + (new TDPoint((float)(parts[i - 1].arm.Length * Math.Cos(Zcur)), (float)(parts[i - 1].arm.Length * Math.Sin(Zcur)), 0));
                    }
                }
                cost = calcCost(tar);
            }
            Console.WriteLine(baseRotation);
            for (int i = 0; i < points.Length; i++) {
                points[i].print();
            }

        }

        public void updatePoints()
        {
            points[0] = new TDPoint(0, 0, 0);
            for (int i = 0; i < parts.Length; i++)
            {
                points[i + 1] = points[i] + parts[i].arm;
            }
        }

        public double calcCost(TDPoint target)
        {
            return Math.Sqrt(Math.Pow(points[points.Length - 1].x - target.x, 2) + Math.Pow(points[points.Length - 1].y - target.y, 2) + Math.Pow(points[points.Length - 1].z - target.z, 2));
        }
    }
}