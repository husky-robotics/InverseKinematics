using System;
namespace InverseKinematics
{
    public struct DualQuaternion
    {
        public Quaternion Re, Du;

        public DualQuaternion(Quaternion Re, Quaternion Du)
        {
            this.Re = Re;
            this.Du = Du;
        }
        public float Length
        {
            get
            {
                return Re.Length;
            }
        }
        public static DualQuaternion operator !(DualQuaternion a)
        {
            return new DualQuaternion(!a.Re, !a.Du);
        }
        public static DualQuaternion operator ~(DualQuaternion a)
        {
            return new DualQuaternion(a.Re, -a.Du);
        }
        public static DualQuaternion operator -(DualQuaternion a)
        {
            return new DualQuaternion(-a.Re, -a.Du);
        }
        public static DualQuaternion operator +(DualQuaternion a, DualQuaternion b)
        {
            return new DualQuaternion(a.Re + b.Re, a.Du + b.Du);
        }
        public static DualQuaternion operator -(DualQuaternion a, DualQuaternion b)
        {
            return a + -b;
        }
        public static DualQuaternion operator *(DualQuaternion a, DualQuaternion b)
        {
            return new DualQuaternion(a.Re * b.Re,(a.Re * b.Du) + (a.Du * b.Re));
        }

        public void print()
        {
            Re.print();
            Du.print();
        }
        
    }
}
