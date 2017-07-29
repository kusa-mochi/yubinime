using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CameraGetPreviewFrame.Models
{
    public struct GakkiParam
    {
        public string gakkiName;
        public double hueFrom;
        public double hueTo;

        public GakkiParam(string name, double from, double to)
        {
            gakkiName = name;
            hueFrom = from;
            hueTo = to;
        }
    }
}
