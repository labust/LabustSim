// Copyright 2022 Laboratory for Underwater Systems and Technologies (LABUST)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using Marus.Visualization;
using Unity.Collections;
using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System;
using Marus.ObjectAnnotation;
using Marus.Utils;
using System.IO;
using UnityEngine.Rendering;
using UnityEngine.Serialization;
using UnityEngine.Rendering.HighDefinition;
using System;
using System.Threading.Tasks;
namespace Marus.Sensors
{

    /// <summary>
    /// Lidar implemented using rays
    /// Implemented using IJobParallelFor on CPU
    /// Can drop performance
    /// </summary>
    public class RaycastLidar : SensorBase
    {
        /// <summary>
        /// Number of horizontal rays
        /// </summary>
        public int WidthRes = 1024;

        /// <summary>
        /// Number of vertical rays
        /// </summary>
        public int HeightRes = 16;

        /// <summary>
        /// Maximum range in meters
        /// </summary>
        public float MaxDistance = 100;

        /// <summary>
        /// Minimum range in meters
        /// </summary>
        public float MinDistance = 0.2f;

        public float VerticalFieldOfView = 30;
        public float HorizontalFieldOfView = 360;

        /// <summary>
        /// PointCloud compute shader
        /// </summary>
        public ComputeShader pointCloudShader;

        /// <summary>
        /// Material set for point cloud display
        /// </summary>
        public Material ParticleMaterial;

        public NativeArray<Vector3> Points;
        public NativeArray<LidarReading> Readings;
        public NativeArray<Vector3> FilteredPoints;
        public NativeArray<LidarReading> FilteredReadings;
        PointCloudManager _pointCloudManager;
        RaycastJobHelper<LidarReading> _raycastHelper;
        Coroutine _coroutine;
        [HideInInspector] public List<LidarConfig> Configs;
        [HideInInspector]  public int ConfigIndex = 0;
        [SerializeField] [HideInInspector] public NativeArray<(float, float)> _rayAngles;
        public List<RayInterval> _rayIntervals;
        public RayDefinitionType _rayType;

        private PointCloudSegmentationSaver _saver;
        private bool saverExists;

        [HideInInspector]
        private Dictionary<int, int> colliderLayer;

        public LayerMask blackHoleLayers;
        // modified for fog
       //**********************************************************************
        public Volume volume;
        Fog fog;
        private float fogAttenuationDistance;

        RainIntensity rain;

        private float rainIntensity;

        int counter;
        

        void Start()
        {
            int totalRays = WidthRes * HeightRes;
            colliderLayer = new Dictionary<int, int>();

            if(blackHoleLayers != 0)
            {
                GameObject[] objects = Helpers.FindGameObjectsInLayerMask(blackHoleLayers);

                foreach (var obj in objects)
                {
                    Collider instId = obj.GetComponent<Collider>();
                    if(instId)
                    {
                        colliderLayer.Add(instId.GetInstanceID(),instId.gameObject.layer);
                    }
                }
            }

            _saver = GetComponent<PointCloudSegmentationSaver>();
            saverExists = _saver is not null && _saver.isActiveAndEnabled == true;
            InitializeRayArray();
            Points = new NativeArray<Vector3>(totalRays, Allocator.Persistent);
            Readings = new NativeArray<LidarReading>(totalRays, Allocator.Persistent);

            var directionsLocal = RaycastJobHelper.CalculateRayDirections(_rayAngles);
            _raycastHelper = new RaycastJobHelper<LidarReading>(gameObject, 
                directionsLocal, 
                OnLidarHit, 
                OnFinish, 
                maxDistance:MaxDistance, 
                minDistance:MinDistance,
                sampleFrequency:SampleFrequency);

            if (ParticleMaterial == null)
                ParticleMaterial = PointCloudManager.FindMaterial("PointMaterial");
            if (pointCloudShader == null)
                pointCloudShader = PointCloudManager.FindComputeShader("PointCloudCS");

            _pointCloudManager = PointCloudManager.CreatePointCloud(gameObject, name + "_PointCloud", totalRays, ParticleMaterial, pointCloudShader);
            _coroutine = StartCoroutine(_raycastHelper.RaycastInLoop());

            // fog
            volume.profile.TryGet(out fog);
            fogAttenuationDistance = fog.meanFreePath.value;

            // rain
            rain = FindObjectOfType<RainIntensity>();

            if (rain != null)
            {
                rainIntensity = rain.Intensity;
            }
            else{
                rainIntensity = 0;
            }
            

        }
       
        protected override void SampleSensor()
        {
            _pointCloudManager.UpdatePointCloud(Points);
            _raycastHelper.SampleFrequency = SampleFrequency;
        }
        private int fileCounter = 1; // Brojač datoteka

        private void OnFinish(NativeArray<Vector3> points, NativeArray<LidarReading> readings)
        {
            points.CopyTo(this.Points);
            readings.CopyTo(this.Readings);
            Log(new {points});
            hasData = true;

            // filter detected points and readings in foggy conditions

            (NativeArray<LidarReading> fogFilteredReadings, NativeArray<Vector3> fogFilteredPoints) = FogFilteredReadings(readings, points);
            
          
            (NativeArray<LidarReading> rainFilteredReadings, NativeArray<Vector3> rainFilteredPoints) = RainFilteredReadings(fogFilteredReadings, fogFilteredPoints);
            

            // save intensity in files...

            string folderPath = @"C:\Users\lovre\Git\LidarData"; // Putanja do foldera
            string fileName = $"intenzitet{fileCounter}.txt"; // Ime datoteke
            string filePath = Path.Combine(folderPath, fileName); // Putanja do datoteke

            // Ako folder ne postoji, kreiraj ga
            if (!Directory.Exists(folderPath))
            {
                Directory.CreateDirectory(folderPath);
            }

            // Upisivanje podataka u datoteku
            var combined = rainFilteredPoints.Zip(rainFilteredReadings, (p, i) => new { p, i });
            using (StreamWriter writer = File.CreateText(filePath))
            {   
                
               foreach (var point_int in combined)
                {
                  writer.WriteLine($"x: {point_int.p.x}, y: {point_int.p.y}, z: {point_int.p.z}, I: {point_int.i.Intensity}");
                   
                }
            }
            
            fileCounter++; // increase counter value
          
            string folderPathPCD = @"C:\Users\lovre\Git\PCD"; // set path to folder
            string fileNamePCD = $"point_cloud{fileCounter}.pcd"; // file name
            string filePathPCD = Path.Combine(folderPathPCD, fileNamePCD); // combine
            if (!Directory.Exists(folderPathPCD))
            {
                Directory.CreateDirectory(folderPathPCD);
            }
            PCDSaver.WriteToPcdFileWithIntensity(filePathPCD, rainFilteredPoints.ToList(), rainFilteredReadings.ToList());
            //ExportToPCD(fileName, rainFilteredPoints, rainFilteredReadings);
    
        }

        private static void WriteToPcdFileWithIntensity(string filePath, NativeArray<Vector3> pointcloud, NativeArray<LidarReading> lidarReadings)
        {
            string metadata = "VERSION .7\n" +
                "FIELDS x y z intensity\n" +
                "SIZE 4 4 4 4\n" +
                "TYPE F F F U\n" +
                "COUNT 1 1 1 1\n" +
                "WIDTH " + pointcloud.Length.ToString() + "\n" +
                "HEIGHT 1\n" +
                "VIEWPOINT 0 0 0 1 0 0 0\n" +
                "POINTS " + pointcloud.Length.ToString() + "\n" +
                "DATA binary\n";

            var pointSize = 16; // modified
            File.WriteAllText(filePath, metadata);
            using (var fileStream = new FileStream(filePath, FileMode.Append, FileAccess.Write, FileShare.None))
            using (var bw = new BinaryWriter(fileStream))
            {   
                byte[] bytes = new byte[pointcloud.Length * pointSize];
                for(int i = 0; i < pointcloud.Length; i++)
                {   
                    //pointSize = (int)lidarReadings[i].Intensity/255;
                    Buffer.BlockCopy( BitConverter.GetBytes( pointcloud[i].x ), 0, bytes, i*pointSize, 4 );
                    Buffer.BlockCopy( BitConverter.GetBytes( pointcloud[i].z ), 0, bytes, i*pointSize + 4, 4 );
                    Buffer.BlockCopy( BitConverter.GetBytes( pointcloud[i].y ), 0, bytes, i*pointSize + 8, 4 );
                    Buffer.BlockCopy( BitConverter.GetBytes( lidarReadings[i].Intensity ), 0, bytes, i*pointSize + 12, 4 );
                }
                bw.Write(bytes);
            }
        }
        

        private LidarReading OnLidarHit(RaycastHit hit, Vector3 direction, int index)
        {
            var reading = new LidarReading();
            (int, int) value;
            if (saverExists)
            {
                if (_saver.objectClassesAndInstances.TryGetValue(hit.colliderInstanceID, out value))
                {
                    reading.ClassId = value.Item1;
                    reading.InstanceId = value.Item2;
                }
            }
            if (hit.colliderInstanceID is not 0)
            {
                reading.IsValid = true;
            }
            if(colliderLayer.Count > 0)
            {
                int layer;
                if(colliderLayer.TryGetValue(hit.colliderInstanceID, out layer))
                {
                    reading.IsValid = false;
                }
            }
            //modified --------------------------------------------------------------------------
            var distance = hit.distance;
           // reading.Intensity = 255;
            
            float dotProduct = Vector3.Dot(hit.normal, direction);

            // calculate magnitude of direction and hit.normal
            float magnitudeNormale = hit.normal.magnitude;
            float magnitudeDirection = direction.magnitude;

            // calculate cosine between hit.normale and direction
            float cosine = Math.Abs(dotProduct / (magnitudeNormale * magnitudeDirection));
            
            double number = Math.Round(255f / Math.Pow(distance,2))*cosine;
            
            int constant = 700;
            reading.Intensity = (ushort)(Math.Round(constant*(255f / Math.Pow(2*distance,2))));
            if (reading.Intensity > 255){
                reading.Intensity = 255;
            }

            //-----------------------------------------------------------------------------------------
            // adjusting intensity in foggy condition
            
            reading = FogIntensityReduction(reading);

            // adjusting intensity in rainy condition
            
            if (rain != null)
            {
                reading = RainIntensityReduction(reading);
            }
            
            /*
            //filtering readings in foggy condition in terms of detection range
            double dist =  distance;
            double detectionRange = CalcDetectionRange(distance);
            if(detectionRange < distance){
                reading.IsValid = false;
            }
            */

            return reading;
        }

        private LidarReading FogIntensityReduction(LidarReading reading){
            
            var eta_fog = (1.01 + 0.15*fogAttenuationDistance)/80;
            if (eta_fog > 1)
            {
                eta_fog = 1;
            }
            reading.Intensity = (ushort)(reading.Intensity*eta_fog);
            return reading;
       }

          private LidarReading RainIntensityReduction(LidarReading reading){
            
            var eta_rain = (18.585 - 2.356*rainIntensity + 0.062*Math.Pow(rainIntensity,2))/ 18.585;
            if (rainIntensity > 10)
            {
                eta_rain = 0;
            }
            reading.Intensity = (ushort)(reading.Intensity*eta_rain);
            return reading;
       }

       /*

        private double CalcDetectionRange(double dist){
            double range = 3.653 + 1.497*fogAttenuationDistance - 0.00258*Math.Pow(fogAttenuationDistance,2);
            if(range > 200){
                range = 200;
            }
            return range;
            
       }
        */

        private (NativeArray<LidarReading>, NativeArray<Vector3>) FogFilteredReadings(NativeArray<LidarReading> readings, NativeArray<Vector3> points){

            double p = 0.00032*Math.Pow(fogAttenuationDistance,2) - 0.0164*fogAttenuationDistance + 0.167; // calculate percentage of points that remain
            if (p > 1){
                return (readings, points);
            }
            if (p <0){
                p = 0;
            }
            int n = readings.Length;
            int m = (int)Math.Round(n*p);

            FilteredReadings = new NativeArray<LidarReading>(m, Allocator.Temp);
            FilteredPoints = new NativeArray<Vector3>(m, Allocator.Temp);            
            
            List<int> randomNumbers = new List<int>();
            HashSet<int> set = new HashSet<int>();

            System.Random rand = new System.Random(Guid.NewGuid().GetHashCode());

            while (set.Count < m)
            {
                int randomNumber = rand.Next(0,n);
                set.Add(randomNumber);
            }

            // Pretvori HashSet u listu
            List<int> randomNumberList = new List<int>(set);

            int i = 0;

            foreach(int r in randomNumberList){
                FilteredReadings[i] = readings[r];
                FilteredPoints[i] = points[r];
                i++;
            }
           
            return (FilteredReadings, FilteredPoints);
        }

        private (NativeArray<LidarReading>, NativeArray<Vector3>) RainFilteredReadings(NativeArray<LidarReading> readings, NativeArray<Vector3> points){
            if (rain = null){
                return (readings, points);
            }

            double p = (270 - 12.384*rainIntensity - 1.361*Math.Pow(rainIntensity,2))/270; // calculate percentage of points that remain
            if (p < 0){
                p = 0;
            }
            if (p >= 1){
                return (readings, points);
            }
            int n = readings.Length;
            int m = (int)Math.Round(n*p);

            FilteredReadings = new NativeArray<LidarReading>(m, Allocator.Temp);
            FilteredPoints = new NativeArray<Vector3>(m, Allocator.Temp);            
            
            List<int> randomNumbers = new List<int>();
            HashSet<int> set = new HashSet<int>();

            System.Random rand = new System.Random(Guid.NewGuid().GetHashCode());

            while (set.Count < m)
            {
                int randomNumber = rand.Next(0,n);
                set.Add(randomNumber);
            }

            // Pretvori HashSet u listu
            List<int> randomNumberList = new List<int>(set);

            int i = 0;

            foreach(int r in randomNumberList){
                FilteredReadings[i] = readings[r];
                FilteredPoints[i] = points[r];
                i++;
            }
           
            return (FilteredReadings, FilteredPoints);
        }

        /// <summary>
        /// This method applies parameters and configuration
        /// Active configuration is selected using dropdown from inspector
        /// </summary>
        public void ApplyLidarConfig()
        {
            if (_rayAngles.IsCreated)
            {
                _rayAngles.Dispose();
            }
            var cfg = Configs[ConfigIndex];
            MaxDistance = cfg.MaxRange;
            MinDistance = cfg.MinRange;
            WidthRes = cfg.HorizontalResolution;
            HeightRes = cfg.VerticalResolution;
            HorizontalFieldOfView = cfg.HorizontalFieldOfView;
            VerticalFieldOfView = cfg.VerticalFieldOfView;
            SampleFrequency = cfg.Frequency;
            _rayType = cfg.Type;
            _rayIntervals = cfg.RayIntervals;
            if (cfg.Type == RayDefinitionType.Angles)
            {
                HeightRes = cfg.ChannelAngles.Count;
            }
            else if (cfg.Type == RayDefinitionType.Intervals)
            {
                if (_rayIntervals is null)
                {
                    _rayIntervals = new List<RayInterval>();
                }
                else
                {
                    if (_rayIntervals.Count > 0)
                    {
                        HeightRes = _rayIntervals.Sum(x => x.NumberOfRays);
                        VerticalFieldOfView = _rayIntervals.Last().EndingAngle - _rayIntervals.First().StartingAngle;
                    }
                }
            }
        }

        /// <summary>
        /// Initializes ray directions from ray angles, custom ray intervals or uniform distribution.
        /// These directions emulate lidar vertical array rotating to get the surrounding pointcloud
        /// </summary>
        public void InitializeRayArray()
        {
            var cfg = Configs[ConfigIndex];
            if (cfg.Type == RayDefinitionType.Intervals)
            {
                var angles = RaycastJobHelper.InitVerticalAnglesFromIntervals(_rayIntervals, WidthRes, HorizontalFieldOfView);
                _rayAngles = RaycastJobHelper.InitCustomRays(angles, cfg.HorizontalResolution, HorizontalFieldOfView);
            }
            else if (cfg.Type == RayDefinitionType.Uniform)
            {
                _rayAngles = RaycastJobHelper.InitUniformRays(WidthRes, HeightRes, HorizontalFieldOfView, VerticalFieldOfView);
            }
            else if (cfg.Type == RayDefinitionType.Angles)
            {
                HeightRes = cfg.ChannelAngles.Count;
                _rayAngles = RaycastJobHelper.InitCustomRays(cfg.ChannelAngles, cfg.HorizontalResolution, HorizontalFieldOfView);
            }
        }

        void OnDestroy()
        {
            _raycastHelper?.Dispose();
            Points.Dispose();
            Readings.Dispose();
            _rayAngles.Dispose();
          
            

        }
    }

    /// <summary>
    /// Class containing all needed lidar properties.
    /// </summary>
    [System.Serializable]
    public class LidarConfig
    {
        public string Name;
        public RayDefinitionType Type;
        public float Frequency;
        public string FrameId;
        public float MaxRange;
        public float MinRange;
        public int HorizontalResolution;
        public int VerticalResolution;
        public float HorizontalFieldOfView;
        public float VerticalFieldOfView;
        public List<float> ChannelAngles;
        public List<RayInterval> RayIntervals;
    }

    /// <summary>
    /// Uniform - rays are distributed uniformly
    /// Intervals - rays are distributed based on given angle intervals with number of rays
    /// Angles - rays are distributed based on given angles of vertical rays
    /// </summary>
    public enum RayDefinitionType
    {
        Uniform,
        Intervals,
        Angles
    }

    public struct LidarReading
    {
        public int ClassId;
        public int InstanceId;
        public ushort Intensity;
        public bool IsValid;
    }

    /// <summary>
    /// Ray interval definition
    /// </summary>
    [Serializable]
	public class RayInterval
	{
		public float StartingAngle;
		public float EndingAngle;
		public int NumberOfRays;
	}
}
