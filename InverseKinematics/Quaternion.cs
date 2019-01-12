using System;
namespace InverseKinematics
{
    public struct Quaternion
    {
        public float r, i, j, k;
        public Quaternion(float r, float i, float j, float k)
        {
            this.r = r;
            this.i = i;
            this.j = j;
            this.k = k;
        }
        public Quaternion Normalized()
        {
            float len = Length;
            Quaternion res;
            res.r = r / len;
            res.i = i / len;
            res.j = j / len;
            res.k = k / len;
            return res;
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
            Quaternion res;
            res.r = a.r;
            res.i = -a.i;
            res.j = -a.j;
            res.k = -a.k;
            return res;
        }
        public static Quaternion operator -(Quaternion a)
        {
            Quaternion res;
            res.r = -a.r;
            res.i = -a.i;
            res.j = -a.j;
            res.k = -a.k;
            return res;
        }
        public static Quaternion operator +(Quaternion a, Quaternion b)
        {
            Quaternion res;
            res.r = a.r + b.r;
            res.i = a.i + b.i;
            res.j = a.j + b.j;
            res.k = a.k + b.k;
            return res;
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
            return res;
        }
        public static Quaternion operator *(float len, Quaternion a)
        {
            //Quaternion res = new Quaternion();
            //res.r *= len;
            //res.i *= len;
            //res.j *= len;
            //res.k *= len;
            ////return res;
            return new Quaternion(a.r * len, a.i * len, a.j * len, a.k * len);
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
