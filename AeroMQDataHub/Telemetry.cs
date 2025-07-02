using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AeroMQDataHub
{
    public class Telemetry
    {
        public double Roll { get; set; }
        public double Pitch { get; set; }
        public double Yaw { get; set; }
        public double Volts { get; set; }
        public double Amps { get; set; }
    }
}
