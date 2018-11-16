using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ConsoleApplicationTest1
{
    class Program
    {
        int originX;
        int originY;
        Point origin;
        int angleIncrement = 10;
        int[] preAngle = new int[4];
        int armLength = 75;

        static void Main(string[] args)
        {
            //For 2D
            //DrawingWindow window = new DrawingWindow();


            //For 3D
            _3D window = new _3D();



            Application.Run(window);

        }


        public static double rad(float angle)
        {
            return (Math.PI / 180) * angle;
        }
    }
}
