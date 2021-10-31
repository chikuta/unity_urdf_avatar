using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using ImageMsg = RosMessageTypes.Sensor.ImageMsg;

namespace UrdfAvatar
{
    class ImageReceiver : MonoBehaviour
    {
        [SerializeField] string TopicName = "/pose_estimation/pose_image";
        [SerializeField] RenderTexture TargetRender = null;


        void Start()
        {
            ROSConnection.instance.Subscribe<ImageMsg>(TopicName, ReceiveImageMessage);
        }


        void Update()
        {
        }

        void ReceiveImageMessage(ImageMsg msg)
        {
        }
    }
 }