using mVector3 = Geometry.Vector3;
using uVector3 = UnityEngine.Vector3;
using mQuaternion = Geometry.Quaternion;
using uQuaternion = UnityEngine.Quaternion;
using mPoint = Geometry.Point;
using mColor = Std.ColorRGBA;
using UnityEngine;

namespace Labust.Networking
{
    public static class MessageExtensions
    {
        public static mVector3 AsMsg(this uVector3 vec) =>
            new mVector3() { X = vec.x, Y = vec.y, Z = vec.z };

        public static uVector3 AsUnity(this mVector3 vec) =>
            new uVector3((float)vec.X, (float)vec.Y, (float)vec.Z);

        public static mQuaternion AsMsg(this uQuaternion vec) =>
            new mQuaternion { X = vec.x, Y = vec.y, Z = vec.z, W = vec.w };

        public static uQuaternion AsUnity(this mQuaternion vec) =>
            new uQuaternion((float)vec.X, (float)vec.Y, (float)vec.Z, (float)vec.W);

        public static uVector3 AsUnity(this mPoint point) =>
            new uVector3((float)point.X, (float)point.Y, (float)point.Z);

        public static Color AsUnity(this mColor color) =>
            new Color(color.R, color.G, color.B, color.A);
    }
}
