using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace InverseKinematics
{
    public partial class Point3D : Component
    {
        public Point3D()
        {
            InitializeComponent();
        }

        public Point3D(IContainer container)
        {
            container.Add(this);

            InitializeComponent();
        }
    }
}
