using UnityEngine;
// using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using PoseStampedMsg = RosMessageTypes.Geometry.PoseStampedMsg;


namespace UrdfAvatar
{
    class HandPoseReceiver : MonoBehaviour
    {
        [SerializeField] string TopicName = "/pose_estimation/rhand";
        [SerializeField] GameObject TargetPose = null;
        [SerializeField] Transform Origin = null;
        [SerializeField] float MappingScale = 1.5f;

        void Start()
        {
            ROSConnection.instance.Subscribe<PoseStampedMsg>(TopicName, ReceivePoseMessage);
        }

        void ReceivePoseMessage(PoseStampedMsg msg)
        {
            Vector3 position = msg.pose.position.From<FLU>();
            Quaternion quat = msg.pose.orientation.From<FLU>();
            position *= MappingScale;
            TargetPose.transform.position = Origin.transform.position;
            TargetPose.transform.rotation = Origin.transform.rotation;
            TargetPose.transform.Translate(position);
        }
    }
}