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
        public int? Mah { get; set; }
        public int Fix { get; set; }
        public int Sats { get; set; }
        public double Lat { get; set; }
        public double Lon { get; set; }
        public double Alt { get; set; }
        public double? GpsSpeedMs { get; set; }
        public double? GpsCourseDeg { get; set; }
        public bool? Armed { get; set; }
        public string? Mode { get; set; }
        public bool? GpsRescue { get; set; }
        public int? CycleTimeUs { get; set; }
        public int? RcRoll { get; set; }
        public int? RcPitch { get; set; }
        public int? RcYaw { get; set; }
        public int? RcThrottle { get; set; }
        public double? BaroAltM { get; set; }
        public double? VSpeedMs { get; set; }
    }
}
