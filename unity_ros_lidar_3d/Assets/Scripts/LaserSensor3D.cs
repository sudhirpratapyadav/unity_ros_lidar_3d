using System;
using System.Collections.Generic;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;

public class LaserSensor3D
{
    float RangeMetersMin;
    float RangeMetersMax;

    float fov_horizontal;
    float fov_vertical;

    float angularResolution_vertical;
    float angularResolution_horizontal;

    int NumMeasurementsPerScan_h;
    int NumMeasurementsPerScan_v;

    float[] scanAngleArray_h;
    float[] scanAngleArray_v;
    
    string FrameId;

    uint numPoints;
    uint raw_data_len;
    byte[] raw_data;

    GameObject laser_sensor_link;

    // float avg_time;
    // int total_counts;


    public LaserSensor3D(GameObject _laser_sensor_link, float _RangeMetersMin, float _RangeMetersMax, float _fov_horizontal, float _fov_vertical, float _angularResolution_vertical, float _angularResolution_horizontal)
    {
        RangeMetersMin = _RangeMetersMin;
        RangeMetersMax = _RangeMetersMax;
        fov_horizontal = _fov_horizontal<=360?_fov_horizontal:360;
        fov_vertical = _fov_vertical<=360?_fov_vertical:360;;
        angularResolution_vertical = _angularResolution_vertical;
        angularResolution_horizontal = _angularResolution_horizontal;

        laser_sensor_link = _laser_sensor_link;

        FrameId = laser_sensor_link.name;

        float ScanAngleStart_h = -fov_horizontal/2;
        float ScanAngleEnd_h = fov_horizontal/2;
        float ScanAngleStart_v = -fov_vertical/2;
        float ScanAngleEnd_v = fov_vertical/2;

        NumMeasurementsPerScan_h = Mathf.FloorToInt((ScanAngleEnd_h - ScanAngleStart_h) / angularResolution_horizontal) + 1;
        NumMeasurementsPerScan_v = Mathf.FloorToInt((ScanAngleEnd_v - ScanAngleStart_v) / angularResolution_horizontal) + 1;

        if (fov_horizontal==360)
        {
            NumMeasurementsPerScan_h = NumMeasurementsPerScan_h -1;
        }
        if (fov_vertical==360)
        {
            NumMeasurementsPerScan_v = NumMeasurementsPerScan_v -1;
            
        }
        
        scanAngleArray_h = new float[NumMeasurementsPerScan_h];
        for (int i = 0; i < NumMeasurementsPerScan_h; i++)
        {
            scanAngleArray_h[i] = ScanAngleStart_h + i * angularResolution_horizontal;
        }

        NumMeasurementsPerScan_v = Mathf.FloorToInt((ScanAngleEnd_v - ScanAngleStart_v) / angularResolution_vertical) + 1;
        scanAngleArray_v = new float[NumMeasurementsPerScan_v];
        for (int i = 0; i < NumMeasurementsPerScan_v; i++)
        {
            scanAngleArray_v[i] = ScanAngleStart_v + i * angularResolution_vertical;
        }

        numPoints = (uint)(NumMeasurementsPerScan_h*NumMeasurementsPerScan_v);

        raw_data_len = 16*numPoints;
        raw_data = new byte[raw_data_len];

        // avg_time = 0;
        // total_counts = 0;
    }

    public PointCloud2Msg getScanMsg()
    {

        float startTime = Time.realtimeSinceStartup;

        Transform sensor_transform = laser_sensor_link.transform;

        int raw_data_indx = 0;
        for (int i = 0; i < NumMeasurementsPerScan_h; i++)
        {
            for (int j = 0; j < NumMeasurementsPerScan_v; j++)
            {
                var theta = Mathf.Deg2Rad*scanAngleArray_h[i];
                var psi = Mathf.Deg2Rad*scanAngleArray_v[j];
                var local_dir_vec = new Vector3(Mathf.Cos(psi)*Mathf.Sin(theta), -Mathf.Sin(psi), Mathf.Cos(psi)*Mathf.Cos(theta));
                var directionVector = sensor_transform.rotation * local_dir_vec;

                var measurementStart = RangeMetersMin * directionVector + sensor_transform.position;
                var measurementRay = new Ray(measurementStart, directionVector);
                var foundValidMeasurement = Physics.Raycast(measurementRay, out var hit, RangeMetersMax);

                // Only record measurement if it's within the sensor's operating range
                if (foundValidMeasurement)
                {
                    BitConverter.GetBytes(hit.point.z-sensor_transform.position.z).CopyTo(raw_data, raw_data_indx * 16);
                    BitConverter.GetBytes(-(hit.point.x-sensor_transform.position.x)).CopyTo(raw_data, raw_data_indx * 16+4);
                    BitConverter.GetBytes(hit.point.y-sensor_transform.position.y).CopyTo(raw_data, raw_data_indx * 16+8);
                    BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);
                }
                else
                {
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16);
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+4);
                    BitConverter.GetBytes(float.MaxValue).CopyTo(raw_data, raw_data_indx * 16+8);
                    BitConverter.GetBytes(0.0f).CopyTo(raw_data, raw_data_indx * 16 + 12);

                }
                ++raw_data_indx;
            }
        }
        

        var timestamp = new TimeStamp(Clock.time);
        
        var msg = new PointCloud2Msg
        {
            header = new HeaderMsg
            {
                frame_id = FrameId,
                stamp = new TimeMsg
                {
                    sec = timestamp.Seconds,
                    nanosec = timestamp.NanoSeconds,
                }
            },
            height = 1,
            width = numPoints,
            fields = new PointFieldMsg[]
            {
                new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("i", 12, PointFieldMsg.FLOAT32, 1)
            },
            is_bigendian = false,
            point_step = 16,
            row_step = raw_data_len,
            data = raw_data,
            is_dense = false,
        };

        // float endTime = Time.realtimeSinceStartup;
        // float elapsedTime = 1000*(endTime - startTime);

        // float total_time = (total_counts*avg_time + elapsedTime);
        // total_counts = total_counts + 1;
        // avg_time = total_time/total_counts;

        // Debug.Log("getScanMsg() Execution Time (curr, avg): (" + elapsedTime.ToString("F4") +", "+ avg_time.ToString("F4")+ ") ms");

        return msg;
    }
}
