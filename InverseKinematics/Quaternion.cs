using System;
namespace InverseKinematics
{
    public struct Quaternion
    {
        public float r, i, j, k;
        public int Xmin, Xmax, Ymin, Ymax, Zmin, Zmax;

        public Quaternion(float r, float i, float j, float k)
        {
            this.r = r;
            this.i = i;
            this.j = j;
            this.k = k;
            Xmin = 1;
            Xmax = 1;
            Ymin = 1;
            Ymax = 1;
            Zmin = 1;
            Zmax = 1;
        }

        public Quaternion(float r, float i, float j, float k, int Xmin, int Xmax, int Ymin, int Ymax, int Zmin, int Zmax)
        {
            this.r = r;
            this.i = i;
            this.j = j;
            this.k = k;
            this.Xmin = Xmin;
            this.Xmax = Xmax;
            this.Ymin = Ymin;
            this.Ymax = Ymax;
            this.Zmin = Zmin;
            this.Zmax = Zmax;
        }

        public Quaternion Normalized()
        {
            float len = Length;
            //Quaternion res;
            //res.r = r / len;
            //res.i = i / len;
            //res.j = j / len;
            //res.k = k / len;
            return new Quaternion(r / len, i / len, j / len, k / len, Xmin, Xmax, Ymin, Ymax, Zmin, Zmax);
            //return res;
        }
        public float Length
        {
            get
            {
                return (float)Math.Sqrt(r * r + i * i + j * j + k * k);
            }
        }
        public static Quaternion operator !(Quaternion a)
        {
            //Quaternion res;
            //res.r = a.r;
            //res.i = -a.i;
            //res.j = -a.j;
            //res.k = -a.k;
            //res.Xmin = a.Xmin;
            //res.Xmax = a.Xmax;
            return new Quaternion(a.r, -a.i, -a.j, -a.k, a.Xmin, a.Xmax, a.Ymin, a.Ymax, a.Zmin, a.Zmax);
            //return res;
        }


        public static Quaternion operator -(Quaternion a)
        {
            //Quaternion res;
            //res.r = -a.r;
            //res.i = -a.i;
            //res.j = -a.j;
            //res.k = -a.k;
            //res.min = a.min;
            //res.max = a.max;
            //return res;
            return new Quaternion(-a.r, -a.i, -a.j, -a.k, a.Xmin, a.Xmax, a.Ymin, a.Ymax, a.Zmin, a.Zmax);
        }
        public static Quaternion operator +(Quaternion a, Quaternion b)
        {
            //Quaternion res;
            //res.r = a.r + b.r;
            //res.i = a.i + b.i;
            //res.j = a.j + b.j;
            //res.k = a.k + b.k;
            //res.min = 0;
            //res.max = 360;
            //return res;
            return new Quaternion(a.r + b.r, a.i + b.i, a.j + b.j, a.k + b.k, 1, 1, 1, 1, 1, 1);
        }
        public static Quaternion operator -(Quaternion a, Quaternion b)
        {
            return a + -b;
        }
        public static Quaternion operator *(Quaternion a, Quaternion b)
        {
            Quaternion res;
            res.r = a.r * b.r - (a.i * b.i + a.j * b.j + a.k * b.k);
            res.i = a.r * b.i + b.r * a.i + a.j * b.k - a.k * b.j;
            res.j = a.r * b.j + b.r * a.j + a.k * b.i - a.i * b.k;
            res.k = a.r * b.k + b.r * a.k + a.i * b.j - a.j * b.i;

            //min max of rot quats = 1
            res.Xmin = a.Xmin * b.Xmin;
            res.Xmax = a.Xmax * b.Xmax;
            res.Ymin = a.Ymin * b.Ymin;
            res.Ymax = a.Ymax * b.Ymax;
            res.Zmin = a.Zmin * b.Zmin;
            res.Zmax = a.Zmax * b.Zmax;
            return res;
        }
        public static Quaternion operator *(float len, Quaternion a)
        {
            return new Quaternion(a.r * len, a.i * len, a.j * len, a.k * len, a.Xmin, a.Xmax, a.Ymin, a.Ymax, a.Zmin, a.Zmax);
        }
        public static Quaternion operator *(Quaternion a, float len)
        {
            return len * a;
        }
        public static Quaternion operator *(double len, Quaternion a)
        {
            return (float)len * a;
        }

        //Dot product
        public static float operator %(Quaternion a, Quaternion b) 
        {
            return a.r * b.r + a.i * b.i + a.j * b.j + a.k * b.k;
        }

        public static Quaternion operator /(Quaternion a, double b)
        {
            return (float)(1 / b) * a;
        }

        public static Quaternion operator *(Quaternion a, double len)
        {
            return (float)len * a;
        }

        public void print()
        {
            Console.WriteLine(r + " " + i + " " + j + " " + k);
        }

        public void zero()
        {
            if(Math.Abs(r) < 0.0001)
            {
                r = 0;
            }
            if (Math.Abs(i) < 0.0001)
            {
                i = 0;
            }
            if (Math.Abs(j) < 0.0001)
            {
                j = 0;
            }
            if (Math.Abs(k) < 0.0001)
            {
                k = 0;
            }
        }
    }
}
