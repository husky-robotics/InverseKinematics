using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InverseKinematics
{
    class IK
    {
        public ForwardKinematics FK;
        public float[] currentPos;
        public float[] destination;
        public float cost;

        public IK(ForwardKinematics FK, float[] destination)
        {
            this.FK = FK;
            this.destination = destination;
            currentPos = FK.getEndpoint();

            cost = calculateCost();
        }

        public float[][] getAngles()
        {
            int iterations = 0;
            while(cost > 0.1)
            {
                directMove();
                cost = calculateCost();
                iterations++;
                //Console.WriteLine(iterations);
                //Console.WriteLine(currentPos[0] + " " + currentPos[1] + " " + currentPos[2]);
            }
            Console.WriteLine(iterations);
            float[][] f = { FK.angleX, FK.angleY, FK.angleZ };
            return f;
        }


        public void directMove()
        {
            float scaleFactor = 1500f;
            float[,] dAngles = returnPartials();
            for (int i = 0; i < FK.armCount; i++)
            {
                //check if in range
                FK.angleX[i] += scaleFactor * dAngles[i, 0];
                if (FK.angleX[i] < FK.quatList[i].Xmin)
                    FK.angleX[i] = (float)(FK.quatList[i].Xmin);
                else if(FK.angleX[i] > FK.quatList[i].Xmax)
                    FK.angleX[i] = (float)(FK.quatList[i].Xmax);

                FK.angleY[i] += scaleFactor * dAngles[i, 1];
                if (FK.angleY[i] < FK.quatList[i].Ymin)
                    FK.angleY[i] = (float)(FK.quatList[i].Ymin);
                else if (FK.angleY[i] > FK.quatList[i].Ymax)
                    FK.angleY[i] = (float)(FK.quatList[i].Ymax);

                FK.angleZ[i] += scaleFactor * dAngles[i, 2];
                if (FK.angleZ[i] < FK.quatList[i].Zmin)
                    FK.angleZ[i] = (float)(FK.quatList[i].Zmin);
                else if (FK.angleZ[i] > FK.quatList[i].Zmax)
                    FK.angleZ[i] = (float)(FK.quatList[i].Zmax);
            }
            FK.solve();
            currentPos = FK.getEndpoint();
        }


        public float[,] returnPartials()
        {
            float delta = 0.001f;
            float startingCost = calculateCost();

            float[,] dAngles = new float[FK.armCount, 3];
            float[][] angles = { FK.angleX, FK.angleY, FK.angleZ };

            for (int rm = 0; rm < FK.armCount; rm++)
            {
                for (int joint = 0; joint < 3; joint++)
                {
                    if (angles[joint][rm] < 0)
                    {
                        dAngles[rm, joint] = 0;
                    }
                    else
                    {
                        angles[joint][rm] += delta;
                        FK.solve();
                        //FK.solve(rm);

                        currentPos = FK.getEndpoint();
                        cost = calculateCost();

                        dAngles[rm, joint] = ((startingCost - cost));
                        angles[joint][rm] -= delta;
                    }
                }
            }
            return dAngles;
        }

        public float calculateCost()
        {
            return (float)(Math.Sqrt((Math.Pow(destination[0] - currentPos[0], 2) + Math.Pow(destination[1] - currentPos[1], 2) + Math.Pow(destination[2] - currentPos[2], 2))));
        }

    }
}
