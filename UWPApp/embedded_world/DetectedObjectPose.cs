using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Newtonsoft.Json;
using RosSharp.RosBridgeClient;
using RosSharp.RosBridgeClient.Messages;
using RosSharp.RosBridgeClient.Messages.Standard;
using RosSharp.RosBridgeClient.Messages.Geometry;

namespace embedded_world
{
    class DetectedObjectPose : Message
    {
        //        Header header
        //  float64 confidence
        //  geometry_msgs/Point32[9] flatBounds
        //  geometry_msgs/Pose pose
        [JsonIgnore]
        public const string RosMessageName = "winml_msgs/DetectedObjectPose";
        public Header header;
        public double confidence;
        public Point[] flatBounds;
        public Pose pose;
        public DetectedObjectPose()
        {
            header = new Header();
            confidence = 0.0;
            flatBounds = new Point[0];
            pose = new Pose();
        }
    }
}
