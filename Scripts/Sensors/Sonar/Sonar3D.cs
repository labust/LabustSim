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

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using Labust.Networking;
using System.Threading;
using Labust.Sensors;
using Labust.Sensors.Core;
using Labust.Visualization;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.UI;
using Sensorstreaming;
using Labust.Core;

namespace Labust.Sensors
{

    /// <summary>
    /// Sonar that cast N rays evenly distributed in configured field of view.
    /// Generates polar and cartesian 2D sonar images. 
    /// Implemented using IJobParallelFor on CPU
    /// Can drop performance
    /// </summary>
    public class Sonar3D : SensorBase
    {

        /// <summary>
        /// Material set for point cloud display
        /// </summary>
        public Material ParticleMaterial;

        /// <summary>
        /// Number of horizontal acoustic rays
        /// </summary>
        public int WidthRes = 256;

        /// <summary>
        /// Number of vertical acoustic rays
        /// </summary>
        public int HeightRes = 256;

        /// <summary>
        /// Maximum sonar range in meters
        /// </summary>
        public float MaxDistance = 30;

        /// <summary>
        /// Starting sonar range in meters
        /// </summary>
        public float MinDistance = 0.6F;

        /// <summary>
        /// Horizontal sonar field of view in degrees
        /// </summary>
        public float HorizontalFieldOfView = 60;

        /// <summary>
        /// Vertical sonar field of view in degrees
        /// </summary>
        public float VerticalFieldOfView = 30;

        /// <summary>
        /// Vertical resolution of the polar sonar image.
        /// Can be set independently of the vertical number of rays or max sonar range.
        /// </summary>
        public int imageHeight = 256;

        /// <summary>
        /// Horizontal resolution of the cartesian sonar image.
        /// Can be set independently of the number of rays or polar image resolution.
        /// </summary>
        public int CartesianXRes = 256;

        /// <summary>
        /// Vertical resolution of the cartesian sonar image.
        /// Can be set independently of the number of rays or polar image resolution.
        /// </summary>
        public int CartesianYRes = 256;

        /// <summary>
        /// Equiangular - rays are distributed uniformly
        /// Equidistant - rays are distributed equidistantly on a horizontal plane
        /// </summary>
        public enum RayDistribution { Equiangular, Equidistant }
        public RayDistribution sonarRayDistribution;

        public enum SonarConfiguration { Custom, TritechGemini1200ik, ArisExplorer3000 }
        public SonarConfiguration sonarConfig;

        /// <summary>
        /// Optional saving of generated polar and cartesian images. 
        /// If enabled, images saved in project_folder/SaveImages/
        /// </summary>
        public bool SaveImages = false;

        /// <summary>
        /// Sonar output gain
        /// </summary>
        public float RayIntensity = 10;

        /// <summary>
        /// Number of raycast rays simulating a single acoustic rays
        /// </summary>
        /// 
        public bool Grid = false;
        public bool AddNoise = false;
        public float NoiseLevel = 0.1f; // General noise intensity
        public float SpeckleLevel = 0.05f; // Speckle noise intensity
        public float RayleighScale = 1.0f; // Rayleigh noise scale
        private System.Random systemRandom = new System.Random();
        int NumRaysPerAccusticRay = 1;

        /// <summary>
        /// Pointcloud copy created on every update
        /// </summary>
        public NativeArray<Vector3> pointsCopy;

        /// <summary>
        /// Cartesian and polar raw image arrays for canvas display
        /// </summary>
        public RawImage sonarDisplay, sonarPhotoDisplay, sonarCartesianDisplay;

        /// <summary>
        /// Cartesian and polar texture2D arrays
        /// </summary>
        public Texture2D sonarImage, sonarPhotoImage, sonarCartesianImage;

        public NativeArray<SonarReading> sonarData;
        int imageCount = 1;
        double thetha;
        double r;
        public ComputeShader pointCloudShader;
        const float WATER_LEVEL = 0;
        float altitude, pitch;
        PointCloudManager _pointCloudManager;
        RaycastJobHelper<SonarReading> _raycastHelper;
        Coroutine _coroutine;
        Vector3 sonarPosition;
        NativeArray<Vector3> directionsLocal;
        void Start()
        {
            int totalRays = WidthRes * HeightRes * NumRaysPerAccusticRay;

            sonarImage = new Texture2D(WidthRes, imageHeight);
            sonarPhotoImage = new Texture2D(WidthRes, HeightRes);
            sonarCartesianImage = new Texture2D(CartesianXRes, CartesianYRes);

            pointsCopy = new NativeArray<Vector3>(totalRays, Allocator.Persistent);
            sonarData = new NativeArray<SonarReading>(totalRays, Allocator.Persistent);

            //sLoadSonarConfigs();
            InitializeRayArray();

            _raycastHelper = new RaycastJobHelper<SonarReading>(gameObject, directionsLocal, OnSonarHit, OnFinish, MaxDistance);

            _pointCloudManager = PointCloudManager.CreatePointCloud(name + "_PointClout", totalRays, ParticleMaterial, pointCloudShader);

            _coroutine = StartCoroutine(_raycastHelper.RaycastInLoop());
        }

        protected override void SampleSensor()
        {
            _pointCloudManager.UpdatePointCloud(pointsCopy);
            sonarPosition = transform.position;
        }

        private void OnFinish(NativeArray<Vector3> points, NativeArray<SonarReading> sonarReadings)
        {
            points.CopyTo(pointsCopy);
            sonarReadings.CopyTo(sonarData);

            ComposePhotoImage(sonarReadings);
            ComposePolarImage(sonarReadings);
            ComposeCartesianImage(sonarReadings);

            hasData = true;
        }
        /// <summary>
        /// Function for converting hit distance to Y coordinate of the cartesian projection. 
        /// </summary>
        private int DistanceToImageY(float distance)
        {
            if (distance < MaxDistance && distance >= MinDistance)
            {
                int y = (int)Math.Floor(((distance - MinDistance) / (MaxDistance - MinDistance)) * imageHeight);
                return y;
            }
            else
            {
                return 0;
            }
        }
        /// <summary>
        /// Initializes raycast ray directions based on the selection. 
        /// Equidistant distribution projects equidistant points on a horizontal plane (useful for bathymetric or down looking sonar).
        /// Depends on sonar pitch angle and altitude from the bottom plane.
        /// Equiangular distribution sets vertical angles equally.
        /// </summary>
        public void InitializeRayArray()
        {
            if (sonarRayDistribution == RayDistribution.Equidistant)
            {
                //get the pitch angle from the sonar frame
                pitch = transform.eulerAngles.x;

                //sample depth under the sonar for equidistant ray projection
                RaycastHit hit;
                if (UnityEngine.Physics.Raycast(transform.position, Vector3.down, out hit, Mathf.Infinity))
                {
                    altitude = hit.distance;
                }

                directionsLocal = RaycastJobHelper.EquidistantRays(WidthRes, HeightRes, HorizontalFieldOfView, VerticalFieldOfView, altitude, pitch);
            }
            else if (sonarRayDistribution == RayDistribution.Equiangular)
            {
                directionsLocal = RaycastJobHelper.EvenlyDistributeRays(WidthRes, HeightRes, HorizontalFieldOfView, VerticalFieldOfView);
            }
            else
            {
                gameObject.active = false;
                return;
            }
        }

        /// <summary>
        /// Method for setting custom sonar configurations selected from the dropdown sonar list
        /// </summary>
        /*         public void SetSonarConfig()
                {

                } */

        /*         public void LoadSonarConfigs()
                {
                    var jsonText = File.ReadAllText("SonarPresets.json");
                    SonarConfigs = JsonConvert.DeserializeObject<List<SonarConfigs>>(jsonText);
                    sonarObj = target as RaycastSonar;
                    sonarObj.Configs = SonarConfigs;
                    _choices = new string[SonarConfigs.Count];
                    var i = 0;
                    foreach(var cfg in SonarConfigs)
                    {
                        _choices[i++] = cfg.Name;
                    }
                    _configName = _choices[sonarObj.ConfigIndex];
                } */
        void OnDestroy()
        {

            if (pointsCopy.IsCreated) pointsCopy.Dispose();
            if (sonarData.IsCreated) sonarData.Dispose();
            if (directionsLocal.IsCreated) directionsLocal.Dispose();

            _raycastHelper?.Dispose();
        }

        public SonarReading OnSonarHit(RaycastHit hit, Vector3 direction, int i)
        {
            var distance = hit.distance;
            var sonarReading = new SonarReading();
            float intensity = 0;

            //in case of out of range rays add only thermal and speckle noise
            if (distance < MinDistance || hit.point.y > WATER_LEVEL || hit.point == Vector3.zero )
            {
                sonarReading.Valid = false;
                sonarReading.Intensity = 0;
            }
            else
            {
                sonarReading.Valid = true;
                sonarReading.Distance = hit.distance;

                intensity = (RayIntensity / 100) * (float)(Math.Acos(Math.Abs(Vector3.Dot(direction, hit.normal))));

                sonarReading.Intensity = intensity;
            }
            return sonarReading;
        }

        /// <summary>
        /// Function for composing a X-Y "photographic" image from the raycast pointcloud, as seen from the sonar. 
        /// Used optionally. 
        /// </summary>
        private void ComposePhotoImage(NativeArray<SonarReading> reading)
        {
            UnityEngine.Color pixel;
            for (var x = 0; x < WidthRes; x++)
            {
                for (var y = 0; y < HeightRes; y++)
                {
                    if (reading[x * HeightRes + y].Valid)
                    {
                        pixel = new UnityEngine.Color(reading[x * HeightRes + y].Intensity, reading[x * HeightRes + y].Intensity, reading[x * HeightRes + y].Intensity, 1);
                    }
                    else
                    {
                        pixel = new UnityEngine.Color(0, 0, 0, 1);
                    }
                    sonarPhotoImage.SetPixel(x, y, pixel);
                }
            }

            sonarPhotoImage.Apply();
            sonarPhotoDisplay.texture = sonarPhotoImage;
        }
        /// <summary>
        /// Creates a polar sonar image - 2D projection with bearing on X axis and range on Y axis. 
        /// Width and height can be set independently, .png image saving optional.
        /// </summary>
        private void ComposePolarImage(NativeArray<SonarReading> reading)
        {
            UnityEngine.Color pixel;
            int xCoordinate, yCoordinate;
            float currentIntensity; 
            float[] yIntensity = new float[imageHeight];
            for (var x = 0; x < WidthRes; x++)
            {
                //squashing all spatial columns into 2D and adding the intensities
                for (var y = 0; y < HeightRes; y++)
                {
                    currentIntensity = reading[x * HeightRes + y].Intensity;
                    //add sonar noise depending on the a target has been hit or not

                    if(currentIntensity != 0 && AddNoise)
                    {
                        currentIntensity = AddGaussianNoise(currentIntensity);
                        currentIntensity = AddSpeckleNoise(currentIntensity);
                        currentIntensity = AddRayleighNoise(currentIntensity, reading[x * HeightRes + y].Distance);
                    }
                    else if (currentIntensity == 0 && AddNoise)
                    {
                        currentIntensity = AddGaussianNoise(currentIntensity);
                        currentIntensity = AddSpeckleNoise(currentIntensity);
                    }

                    yCoordinate = DistanceToImageY(reading[x * HeightRes + y].Distance);
                    yIntensity[yCoordinate] += currentIntensity;
                }

                //stacking the intensities into corresponding 2D image columns
                for (var y = 0; y < imageHeight; y++)
                {
                    pixel = new UnityEngine.Color(yIntensity[y], yIntensity[y], yIntensity[y], 1);
                    sonarImage.SetPixel(x, y, pixel);
                }
                Array.Clear(yIntensity, 0, yIntensity.Length);
            }

            if (Grid)
            {
                sonarImage = AddGridAndLabels(sonarImage);
            }

            sonarImage.Apply();
            sonarDisplay.texture = sonarImage;

            if (SaveImages)
            {
                byte[] bytes = sonarImage.EncodeToPNG();
                var dirPath = Application.dataPath + "/../SaveImages/";
                if (!Directory.Exists(dirPath))
                {
                    Directory.CreateDirectory(dirPath);
                }
                File.WriteAllBytes(dirPath + "ImagePolar" + imageCount + ".png", bytes);
            }

        }

        /// <summary>
        /// Creates a cartesian sonar image - 2D projection with bearing in cartesian coordinates on X axis and range on Y axis. 
        /// Beamformed based on the angle distribution, removes object distortion.
        /// Width and height can be set independently, .png image saving optional.
        /// </summary>

        private void ComposeCartesianImage(NativeArray<SonarReading> reading)
        {
            UnityEngine.Color pixel;
            int xCoordinate, yCoordinate;

            //populate left side of the swath
            for (var x = CartesianXRes / 2; x > 0; x--)
            {
                for (var y = 0; y < CartesianYRes; y++)
                {
                    thetha = (180 / Math.PI) * Math.Atan2(x, y);
                    r = Math.Sqrt(x * x + y * y);
                    r = r / (float)CartesianYRes * (MaxDistance - MinDistance);

                    if (thetha <= (HorizontalFieldOfView / 2) && r <= MaxDistance && r >= MinDistance)
                    {
                        xCoordinate = (int)Math.Round(((HorizontalFieldOfView / 2) - thetha) / HorizontalFieldOfView * WidthRes);
                        yCoordinate = (int)Math.Round(r / (MaxDistance - MinDistance) * imageHeight);
                        pixel = sonarImage.GetPixel(xCoordinate, yCoordinate);
                        sonarCartesianImage.SetPixel(CartesianXRes / 2 - x, y, pixel);
                    }
                    else
                    {
                        pixel = new UnityEngine.Color(0, 0, 0, 1);
                        sonarCartesianImage.SetPixel(CartesianXRes / 2 - x, y, pixel);
                    }
                }

            }

            //populate right side of the swath
            for (var x = 0; x < CartesianXRes / 2; x++)
            {
                for (var y = 0; y < CartesianYRes; y++)
                {
                    thetha = (180 / Math.PI) * Math.Atan2(x, y);
                    r = Math.Sqrt(x * x + y * y);
                    r = r / CartesianYRes * (MaxDistance - MinDistance);

                    if (thetha <= (HorizontalFieldOfView / 2) && r <= MaxDistance && r >= MinDistance)
                    {
                        xCoordinate = (int)Math.Round((thetha + (HorizontalFieldOfView / 2)) / HorizontalFieldOfView * WidthRes);
                        yCoordinate = (int)Math.Round(r / (MaxDistance - MinDistance) * imageHeight);
                        pixel = sonarImage.GetPixel(xCoordinate, yCoordinate);
                        sonarCartesianImage.SetPixel(x + CartesianXRes / 2, y, pixel);
                    }
                    else
                    {
                        pixel = new UnityEngine.Color(0, 0, 0, 1);
                        sonarCartesianImage.SetPixel(x + CartesianXRes / 2, y, pixel);
                    }
                }
            }

            sonarCartesianImage.Apply();
            sonarCartesianDisplay.texture = sonarCartesianImage;

            if (SaveImages)
            {
                byte[] bytes = sonarCartesianImage.EncodeToPNG();
                var dirPath = Application.dataPath + "/../SaveImages/";
                if (!Directory.Exists(dirPath))
                {
                    Directory.CreateDirectory(dirPath);
                }
                File.WriteAllBytes(dirPath + "Image" + imageCount + ".png", bytes);
                imageCount += 1;
            }
        }

        public Texture2D AddGridAndLabels(Texture2D image)
        {
            //add horizontal grid
            int r = 0;
            for (int i = 1; i < MaxDistance / 10; i++)
            {
                r = DistanceToImageY(i * 10);
                image = DrawLine(image, 0, WidthRes, r, r);
            }

            r = DistanceToImageY(MaxDistance);
            image = DrawLine(image, 0, WidthRes, r, r);

            //add vertical grid
            for (int i = 0; i < 5; i++)
            {
                image = DrawLine(image, i * WidthRes / 4, i * WidthRes / 4, 0, imageHeight);
            }

            return image;
        }

        public Texture2D DrawLine(Texture2D baseImage, int startX, int endX, int startY, int endY)
        {
            UnityEngine.Color color = new UnityEngine.Color(1, 1, 1, 1);

            for (int x = startX; x <= endX; x++)
            {
                for (int y = startY; y <= endY; y++)
                {
                    baseImage.SetPixel(x, y, color);
                }
            }
            return baseImage;
        }

        private float AddGaussianNoise(float intensity)
        {
            float noise = RandomGaussian() * NoiseLevel;
            return Mathf.Clamp(intensity + noise, 0.0f, 1.0f);
        }

        private float RandomGaussian()
        {
            float u1 = 1.0f - (float)systemRandom.NextDouble();
            float u2 = 1.0f - (float)systemRandom.NextDouble();
            return Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        }

        private float AddSpeckleNoise(float intensity)
        {
            float speckle = (1 + RandomGaussian() * SpeckleLevel);
            return Mathf.Clamp(intensity * speckle, 0.0f, 1.0f);
        }

        private float AddRayleighNoise(float intensity, float distance)
        {
            float rayleighNoise = DistanceRayleigh(RayleighScale, distance);
            return Mathf.Clamp(intensity + rayleighNoise, 0.0f, 1.0f);
        }

        private float DistanceRayleigh(float sigma, float r)
        {
            float p_r = (r / sigma * sigma) * Mathf.Exp(-((r * r) / (2 * sigma * sigma)));
            return p_r;
        }
        private float RandomRayleigh(float scale)
        {
            float u = (float)systemRandom.NextDouble();
            return scale * Mathf.Sqrt(-2.0f * Mathf.Log(u));
        }


    }

}