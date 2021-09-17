using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Labust.Logger;
using Newtonsoft.Json;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace Labust.Core
{

    /// <summary>
    /// Class used to control simulation flow
    /// 
    /// Play, pause, restart, quit, OnSave, etc.
    /// </summary>
    public class SimulatorController : MonoBehaviour
    {
        public bool SaveOnExit = true;

        public GameObject PauseMenuUi;
        bool _isRunning;
        float timeScaleBeforePause;

        public string SavesPath => Path.Combine(Application.dataPath, "Saves");

        void Awake()
        {
            _isRunning = true;
            PauseMenuUi.SetActive(false);
            Cursor.lockState = CursorLockMode.Locked;
        }

        void LateUpdate()
        {
            
            if (Input.GetKeyDown(KeyCode.Escape))
            {
                if (_isRunning)
                {
                    Pause();
                }
                else
                {
                    Resume();
                }
            }
        }

        /// <summary>
        /// Called from UI button. Not to be called directly
        /// </summary>
        public void Pause()
        {
            Cursor.lockState = CursorLockMode.Confined;
            timeScaleBeforePause = Time.timeScale;
            Time.timeScale = 0;
            PauseMenuUi.SetActive(true);
            _isRunning = false;
        }

        /// <summary>
        /// Called from UI button. Not to be called directly
        /// </summary>
        public void Resume()
        {
            Cursor.lockState = CursorLockMode.Locked;
            PauseMenuUi.SetActive(false);
            Time.timeScale = timeScaleBeforePause;
            _isRunning = true;
        }

        /// <summary>
        /// Called from UI button. Not to be called directly
        /// </summary>
        public void Restart()
        {
            if (SaveOnExit)
            {
                SaveLogs();
            }
            SceneManager.LoadScene(SceneManager.GetActiveScene().name);
            Resume();
        }

        /// <summary>
        /// Called from UI button. Not to be called directly
        /// </summary>
        public void Exit()
        {
            if (SaveOnExit)
            {
                Save();
            }
            if (Application.isEditor)
            {
                UnityEditor.EditorApplication.isPlaying = false;
            }
            Application.Quit();
        }

        public void Save()
        {
            SaveLogs();
        }

        private void SaveLogs()
        {
            var logs = DataLogger.Instance.ExportAllLogs();
            if (!Directory.Exists(SavesPath))
            {
                Directory.CreateDirectory(SavesPath);
            }
            var currentPath = Path.Combine(SavesPath, $"Scenario{DateTime.Now.ToString("dd-MM-yyyy_hh-mm-ss")}.json");
            var asJson = JsonConvert.SerializeObject(logs, Formatting.Indented, new UnityVectorJsonConverter(), new UnityQuaternionJsonConverter());
            using(var writer = new StreamWriter(currentPath))
            {
                writer.Write(asJson);
            }
        }
    }


    /// <summary>
    /// Custom json converter for Unity Quaternion struct
    /// </summary>   
    internal class UnityQuaternionJsonConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(Quaternion);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            reader.Read();
            var x = reader.ReadAsDouble();
            var y = reader.ReadAsDouble();
            var z = reader.ReadAsDouble();
            var w = reader.ReadAsDouble();
            reader.Read();
            return new Quaternion((float)x, (float)y, (float)z, (float)w);
        }

        public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
        {
            var v = value as Quaternion?;
            writer.WriteStartArray();
            writer.WriteValue(v.Value.x);
            writer.WriteValue(v.Value.y);
            writer.WriteValue(v.Value.z);
            writer.WriteValue(v.Value.w);
            writer.WriteEndArray();
        }
    }

    /// <summary>
    /// Custom json converter for Unity Vector struct
    /// </summary>
    internal class UnityVectorJsonConverter : JsonConverter
    {
        public override bool CanConvert(Type objectType)
        {
            return objectType == typeof(Vector2) || objectType == typeof(Vector3) || objectType == typeof(Vector4);
        }

        public override object ReadJson(JsonReader reader, Type objectType, object existingValue, JsonSerializer serializer)
        {
            reader.Read(); // start array token
            var p1 = reader.ReadAsDouble();
            var p2 = reader.ReadAsDouble();
            if (objectType == typeof(Vector2))
            {
                reader.Read(); // end array token
                return new Vector2((float)p1, (float)p2);
            }
            var p3 = reader.ReadAsDouble();
            if (objectType == typeof(Vector2))
            {
                reader.Read(); // end array token
                return new Vector3((float)p1, (float)p2, (float)p3);
            }
            var p4 = reader.ReadAsDouble();
            if (objectType == typeof(Vector2))
            {
                reader.Read(); // end array token
                return new Vector4((float)p1, (float)p2, (float)p3, (float)p4);
            }
            throw new JsonException();
        }

        public override void WriteJson(JsonWriter writer, object value, JsonSerializer serializer)
        {
            writer.WriteStartArray();
            if (value is Vector2 v2)
            {
                writer.WriteValue(v2[0]);
                writer.WriteValue(v2[1]);
            }
            else if (value is Vector3 v3)
            {
                writer.WriteValue(v3[0]);
                writer.WriteValue(v3[1]);
                writer.WriteValue(v3[2]);
            }
            else if (value is Vector4 v4)
            {
                writer.WriteValue(v4[0]);
                writer.WriteValue(v4[1]);
                writer.WriteValue(v4[2]);
                writer.WriteValue(v4[3]);
            }
            writer.WriteEndArray();
        }
    }
}