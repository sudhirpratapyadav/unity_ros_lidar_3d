using UnityEngine;
using UnityEngine.Serialization;
using System.Collections.Generic;

using RosMessageTypes.Geometry;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;


public class PointCloudPublisher : MonoBehaviour
{
    

    public GameObject laser_sensor_link;
    public string point_cloud_topic = "/point_cloud";
    public string pose_topic = "/laser_scan_pose";

    public float RangeMetersMin = 0;
    public float RangeMetersMax = 1000;

    public float fov_horizontal = 360;
    public float fov_vertical = 45;

    public float angularResolution_vertical = 1;
    public float angularResolution_horizontal = 1;


    ROSConnection ros;
    LaserSensor3D laser_sensor_3d;

    // [SerializeField]
    // double m_PublishRateHz = 20f;
    // double m_LastPublishTimeSeconds;
    // double PublishPeriodSeconds => 1.0f / m_PublishRateHz;
    // bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(point_cloud_topic);
        ros.RegisterPublisher<PoseMsg>(pose_topic);

        laser_sensor_3d = new LaserSensor3D(laser_sensor_link, RangeMetersMin, RangeMetersMax, fov_horizontal, fov_vertical, angularResolution_vertical, angularResolution_horizontal);

        // m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;

    }

    void Update()
    {

        PointCloud2Msg point_cloud_msg = laser_sensor_3d.getScanMsg();

        PoseMsg pose_msg = new PoseMsg
        {
            position = new PointMsg(laser_sensor_link.transform.position.x, laser_sensor_link.transform.position.y, laser_sensor_link.transform.position.z),
            orientation = new QuaternionMsg(laser_sensor_link.transform.rotation.x, laser_sensor_link.transform.rotation.y, laser_sensor_link.transform.rotation.z, laser_sensor_link.transform.rotation.w),
        };

        ros.Publish(point_cloud_topic, point_cloud_msg);
        ros.Publish(pose_topic, pose_msg);

        // if (ShouldPublishMessage)
        // {
        //     PointCloud2Msg msg = laser_sensor_3d.getScanMsg(laser_sensor_link.transform);
        //     ros.Publish(point_cloud_topic, msg);
        //     m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
        // }
    }
}
