﻿using UnityEngine;
using System;
using Sensorstreaming;
using Labust.Networking;

namespace Labust.Sensors.Primitive
{
    public class Sonar : SensorBase<SonarStreamingRequest>
    {
        public float maxRange = 120f;
        public float minBearing = -60f; // 60 degrees to the left
        public float maxBearing = 60f; // 60 degrees to the right
        public Transform trackedObject;

        float range;
        float bearing;

        // Start is called before the first frame update
        void Start()
        {
            //streamHandle = streamingClient?.StreamSonarSensor(cancellationToken:RosConnection.Instance.cancellationToken);
            if (string.IsNullOrEmpty(address))
                address = vehicle.name + "/sonar";
        }

        protected override void SampleSensor()
        {
            var sonar = transform;
            if (Vector3.Distance(sonar.position, trackedObject.position) > 0 &&
                Vector3.Distance(sonar.position, trackedObject.position) < maxRange)
            {
                range = Vector3.Distance(sonar.position, trackedObject.position);
                Debug.DrawRay(sonar.position, transform.TransformDirection(Vector3.up) * range,
                    Color.yellow);
            }
            else
            {
                range = 0;
            }
            Log(new { range, bearing });

            // float xDiff = trackedObject.position.x - sonar.position.x;
            // float zDiff = trackedObject.position.z - sonar.position.z;

            // float angle = (float)Math.Atan2(xDiff, zDiff) * (float)(180/Math.PI);

            // float sonarAngle = sonar.eulerAngles.y;

            // if (sonarAngle>180)
            // {   
            //     sonarAngle -= 360;

            // }

            // bearing = angle - sonarAngle;
            // Debug.Log(sonarAngle);
            // Debug.Log(angle);


            Vector3 targetHeading = new Vector3(trackedObject.position.x, 0, trackedObject.position.z) - 
                                    new Vector3(sonar.position.x, 0, sonar.position.z);
            bearing = -Vector3.SignedAngle(targetHeading, sonar.forward, sonar.up);
            
            if (bearing > maxBearing || bearing < minBearing || range == 0)
            {
                bearing = 0;
                range = 0;
            }

            hasData = true;
        }

        protected async override void SendMessage()
        {
        //     await _streamWriter.WriteAsync(new SonarStreamingRequest
        //     {
        //         Address = address,
        //         Range = range,
        //         Bearing = bearing
        //     });
        //     hasData = false;
        }
    }
}