using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InverseKinematics
{
    public struct TDPoint
    {
        public float x, y, z;

        public TDPoint(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public void set(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }

        public float Length
        {
            get
            {
                return(float)Math.Sqrt(x * x + y * y + z * z);
            }
        }

        public TDPoint Normalized()
        {
            float len = Length;
            return new TDPoint(x / len, y / len, z / len);
        }

        public static Quaternion operator -(TDPoint a, TDPoint b)
        {
            return new Quaternion(0, a.x - b.x, a.y - b.y, a.z - b.z);
        }

        public static TDPoint operator |(TDPoint a, TDPoint b)
        {
            return new TDPoint(a.x - b.x, a.y - b.y, a.z - b.z);
        }

        public static TDPoint operator +(TDPoint a, Quaternion b)
        {
            return new TDPoint(a.x + b.i, a.y + b.j, a.z + b.k);
        }

        public static TDPoint operator +(TDPoint a, TDPoint b)
        {
            return new TDPoint(a.x + b.x, a.y + b.y, a.z + b.z);
        }

        public static TDPoint operator %(TDPoint a, TDPoint b)
        {
            return new TDPoint( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
        }

        public static double operator *(TDPoint a, TDPoint b)
        {
            return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
        }

        public static TDPoint operator *(TDPoint a, float b)
        {
            return new TDPoint(a.x * b, a.y * b, a.z * b);
        }

        public void print()
        {
            Console.WriteLine(x + " " + y + " " + z);
        }

        //check later
        public double angleBetween(TDPoint other)
        {
            float crossed = (this % other).y;
            if(crossed  == 0)
            {
                return 0;
            }
            return (crossed / Math.Abs(crossed)) * (Math.Acos((this * other) / (this.Length * other.Length)));
        }

        public double angleRelativeTo(TDPoint other)
        {
            float crossed = (this % other).z;

            if(crossed == 0)
            {
                return 0;
            }
            else if (crossed > 0)
            {
                return Math.Acos((this * other) / (this.Length * other.Length));
            }
            else
            {
                return 2 * Math.PI - Math.Acos((this * other) / (this.Length * other.Length));
            }

        }
    }

    public struct Point2D
    {
        public float x, y;

        public Point2D(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

        public void print()
        {
            Console.WriteLine(x + " " + y);
        }
    }
}
