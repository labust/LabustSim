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
using Labust.Core;
using Labust.Networking;
using Sensor;
using Sensorstreaming;
using Std;
using UnityEngine;

namespace Labust.Sensors.Primitive
{
    /// <summary>
    /// Imu sensor implementation
    /// </summary>
    [RequireComponent(typeof(ImuSensor))]
    public class ImuROS : SensorStreamer<ImuStreamingRequest>
    {
        ImuSensor sensor;
        void Start()
        {
            sensor = GetComponent<ImuSensor>();
            if (string.IsNullOrEmpty(address))
                address = $"{sensor.vehicle.name}/imu";
            StreamSensor(streamingClient?.StreamImuSensor(cancellationToken:RosConnection.Instance.cancellationToken));
        }

        new void Update()
        {
            hasData = sensor.hasData;
            base.Update();
        }

        protected override async void SendMessage()
        {
            var imuOut = new Imu()
            {
                Header = new Header
                {
                    FrameId = sensor.frameId,
                    Timestamp = TimeHandler.Instance.TimeDouble
                },
                Orientation = sensor.orientation.Unity2Map().AsMsg(),
                AngularVelocity = (-sensor.angularVelocity).Unity2Map().AsMsg(),
                LinearAcceleration = sensor.linearAcceleration.Unity2Map().AsMsg(),
            };
            imuOut.OrientationCovariance.AddRange(sensor.orientationCovariance);
            imuOut.LinearAccelerationCovariance.AddRange(sensor.linearAccelerationCovariance);
            imuOut.AngularVelocityCovariance.AddRange(sensor.angularVelocityCovariance);

            await _streamWriter.WriteAsync(new ImuStreamingRequest
            {
                Data = imuOut,
                Address = address
            });
            hasData = false;
        }
    }
}