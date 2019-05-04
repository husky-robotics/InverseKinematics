using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;

namespace InverseKinematics
{
    class FABRIK
    {
        public TDPoint[] points;
        public double cost;
        ArmQ arm;

        public FABRIK(ArmQ arm) {
            this.arm = arm;
            points = new TDPoint[arm.parts.Length + 1];
            updatePoints();
        }

        //tupules in the future?
        //public FABRIK(ArmQ arm, (float x, float y, float z) target)
        //{
        //    this.target2 = target;
        //    this.arm = arm;
        //    points = new TDPoint[arm.parts.Length + 1];
        //    updatePoints();
        //    this.cost = calcCost();
        //}

        public void converge2D(TDPoint target) {
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
            for (int i = 1; i < points.Length; i++)
            {
                zTot += arm.Zcur[i - 1];
                distance = points[i].Length;
                points[i] = new TDPoint((float)(distance * Math.Cos(zTot)), (float)(distance * Math.Sin(zTot)), 0) + points[i - 1];
            }

            cost = calcCost(target);
            ////////////////////////////////////////////Moving Joints////////////////////////////////////////////
            while (cost > 0.001)
            {
                for (int i = points.Length - 2; i >= 0; i--)
                {
                    if (i == points.Length - 2)
                    {
                        points[i + 1] = tar;
                    }
                    points[i] = points[i + 1] + ((points[i] - points[i + 1]).Normalized() * arm.parts[i].arm.Length);
                }

                points[0].set(0, 0, 0);
                for (int i = 1; i < points.Length; i++)
                {
                    arm.parts[i - 1].arm = ((points[i] - points[i - 1]).Normalized() * arm.parts[i - 1].arm.Length);
                    points[i] = points[i - 1] + arm.parts[i - 1].arm;
                }

                //angle constraints
                double[] changes = new double[points.Length - 1];

                for (int i = points.Length - 2; i > 0; i--) {
                    //TDPoint pre = points[i] | points[i - 1];
                    //TDPoint post = points[i + 1] | points[i];

                    double ang = (points[i] | points[i - 1]).angleRelativeTo(points[i + 1] | points[i]);

                    if (ang < arm.parts[i].Zmin || ang > arm.parts[i].Zmax) {
                        if (ang < arm.parts[i].Zmin) {
                            arm.Zcur[i] = arm.parts[i].Zmin;
                        }
                        else {
                            arm.Zcur[i] = arm.parts[i].Zmax;
                        }
                    }
                    else {
                        arm.Zcur[i] = (float)ang;
                    }
                }
                double angle = (new TDPoint(1, 0, 0)).angleRelativeTo(points[1]);
                if (angle < arm.parts[0].Zmin) {
                    arm.Zcur[0] = arm.parts[0].Zmin;
                }
                else if(angle > arm.parts[0].Zmax) {
                    arm.Zcur[0] = arm.parts[0].Zmax;
                }
                else {
                    arm.Zcur[0] = (float)angle;
                }

                //recalculate points
                {
                    float Zcur = 0;
                    points[0].set(0, 0, 0);
                    for (int i = 1; i < arm.Zcur.Length; i++) {
                        Zcur += arm.Zcur[i - 1];
                        points[i] = points[i - 1] + (new TDPoint((float)(arm.parts[i - 1].arm.Length * Math.Cos(Zcur)), (float)(arm.parts[i - 1].arm.Length * Math.Sin(Zcur)), 0));
                    }
                }
                
                cost = calcCost(tar);
            }

            Console.WriteLine(baseRotation);
            for(int i = 0; i < points.Length; i++) {
                points[i].print();
            }

        }

        //public void recAdjustments(int index, double[] changes, double change)
        //{
        //    for(int i = index; i < changes.Length; i++)
        //    {
        //        changes[i] += change;
        //    }
        //}

        public void updatePoints() {
            points[0] = new TDPoint(0, 0, 0);
            for (int i = 0; i < arm.parts.Length; i++)
            {
                points[i + 1] = points[i] + arm.parts[i].arm;
            }
        }

        //Updates quaternions from the endpoints
        public void quatUpdate() {
            for(int i = 1; i < points.Length; i++)
            {
                arm.parts[i - 1].arm.set(0, points[i].x - points[i-1].x, points[i].y - points[i - 1].y, points[i].z - points[i - 1].z);
            }
        }

        public double calcCost(TDPoint target) {
            return Math.Sqrt(Math.Pow(points[points.Length - 1].x - target.x, 2) + Math.Pow(points[points.Length - 1].y - target.y, 2) + Math.Pow(points[points.Length - 1].z - target.z, 2));
        }
    }
}

