using System;
namespace Quaternion
{
    public struct DualQuaternion
    {
        public Quaternion Re, Im;

        public DualQuaternion(Quaternion Re, Quaternion Im)
        {
            this.Re = Re;
            this.Im = Im;
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
            return new DualQuaternion(!a.Re, !a.Im);
        }
        public static DualQuaternion operator ~(DualQuaternion a)
        {
            return new DualQuaternion(a.Re, -a.Im);
        }
        public static DualQuaternion operator -(DualQuaternion a)
        {
            return new DualQuaternion(-a.Re, -a.Im);
        }
        public static DualQuaternion operator +(DualQuaternion a, DualQuaternion b)
        {
            return new DualQuaternion(a.Re + b.Re, a.Im + b.Im);
        }
        public static DualQuaternion operator -(DualQuaternion a, DualQuaternion b)
        {
            return a + -b;
        }
        public static DualQuaternion operator *(DualQuaternion a, DualQuaternion b)
        {
            return new DualQuaternion(a.Re * b.Re, a.Re * b.Im + b.Im * b.Re);
        }
    }
}
