using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{
    public class Robot_Controller : MonoBehaviour
    {
        public GameObject front_left_wheel;
        public GameObject front_right_wheel;
        public GameObject back_left_wheel;
        public GameObject back_right_wheel;

        private ArticulationBody wFL;
        private ArticulationBody wFR;
        private ArticulationBody wBL;
        private ArticulationBody wBR;

        public float maxLinearSpeed = 3; //  m/s
        public float maxRotationalSpeed = 2;//
        public float wheelRadius = 0.22f; //meters
        public float trackWidth = 0.47f; // meters Distance between tyres
        public float forceLimit = 100;
        public float damping = 50;

        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0f;

        ROSConnection ros;
        private RotationDirection direction;
        private float rosLinear = 0f;
        private float rosAngular = 0f;

        void Start()
        {
            wFL = front_left_wheel.GetComponent<ArticulationBody>();
            wFR = front_right_wheel.GetComponent<ArticulationBody>();
            wBL = back_left_wheel.GetComponent<ArticulationBody>();
            wBR = back_right_wheel.GetComponent<ArticulationBody>();

            SetParameters(wFL);
            SetParameters(wFR);
            SetParameters(wBL);
            SetParameters(wBR);

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("cmd_vel", ReceiveROSCmd);
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }
            RobotInput(rosLinear, -rosAngular); 
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void RobotInput(float speed, float rotSpeed) // m/s and rad/s
        {
            if (speed > maxLinearSpeed)
            {
                speed = maxLinearSpeed;
            }
            if (rotSpeed > maxRotationalSpeed)
            {
                rotSpeed = maxRotationalSpeed;
            }
            float wR_Rotation = (speed / wheelRadius);
            float wL_Rotation = wR_Rotation;
            float wheelSpeedDiff = ((rotSpeed * trackWidth) / wheelRadius);
            if (rotSpeed != 0)
            {
                wR_Rotation = (wR_Rotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                wL_Rotation = (wL_Rotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
            }
            else
            {
                wR_Rotation *= Mathf.Rad2Deg;
                wL_Rotation *= Mathf.Rad2Deg;
            }
            
            SetSpeed(wFL, wL_Rotation);
            SetSpeed(wFR, wR_Rotation);
            SetSpeed(wBL, wL_Rotation);
            SetSpeed(wBR, wR_Rotation);

            // Debug.Log(speed);
            // Debug.Log(rotSpeed);
            // Debug.Log(wL_Rotation); //-174 130
            // Debug.Log(wR_Rotation); //174 130
            // Debug.Log("--------------\n");
        }
    }
}
