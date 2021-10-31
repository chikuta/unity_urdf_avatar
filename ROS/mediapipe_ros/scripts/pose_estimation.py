#!/usr/bin/env python3

import cv2
import pyrealsense2
import numpy as np

import mediapipe as mp
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose

import math
import rospy
import cv_bridge
import sensor_msgs
import geometry_msgs


class Node(object):
    def __init__(self):
        # init ros node
        rospy.init_node("pose_estimation")

        # subscriber for rgb image
        self._sub_rgb = rospy.Subscriber('~rgb', sensor_msgs.msg.Image, self.__sub_rgb_image)
        self._rgb_image = None

        self._sub_rgb_camera_info = rospy.Subscriber('~rgb_camera_info', sensor_msgs.msg.CameraInfo, self.__sub_rgb_camera_info)
        self._camera_info = None

        # subscriber for depth image
        self._sub_depth = rospy.Subscriber('~depth', sensor_msgs.msg.Image, self.__sub_depth_image)
        self._depth_image = None

        # publisher for mediapipe outputs
        self._rhand_pose_pub = rospy.Publisher('~rhand', geometry_msgs.msg.PoseStamped, queue_size=1)
        self._rhand_pose = geometry_msgs.msg.PoseStamped()
        self._rhand_pose.header.frame_id = 'base_link'
        self._rhand_pose.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)

        self._lhand_pose_pub = rospy.Publisher('~lhand', geometry_msgs.msg.PoseStamped, queue_size=1)
        self._lhand_pose = geometry_msgs.msg.PoseStamped()
        self._lhand_pose.header.frame_id = 'base_link'
        self._lhand_pose.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)

        self._pose_image_pub = rospy.Publisher('~pose_image', sensor_msgs.msg.Image, queue_size=1)
        self._pose_depth_image_pub = rospy.Publisher('~pose_depth_image', sensor_msgs.msg.Image, queue_size=1)

        # get rosparam
        self._publish_pose_image = rospy.get_param("~publish_pose_image", True)

        # for process
        self._bridge = cv_bridge.CvBridge()
        # self._pose = mp_pose.Pose(static_image_mode=True, model_complexity=2, enable_segmentation=True, min_detection_confidence=0.5)
        self._pose = mp_pose.Pose(static_image_mode=False, model_complexity=2, enable_segmentation=True, min_detection_confidence=0.5)

    def __sub_rgb_image(self, image):
        # convert image to cv::Mat
        try:
            self._rgb_image = self._bridge.imgmsg_to_cv2(image, "bgr8")
        except Exception as e:
            print(e)
            rospy.logerr(e)

    def __sub_depth_image(self, image):
        # convert image to cv::Mat
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(image, "16UC1")
        except Exception as e:
            rospy.logerr(e)

    def __sub_rgb_camera_info(self, msg):
        self._camera_info = msg

    def _convert_depth_to_coord(self, x, y, depth, camera_info):
        intrinsics = pyrealsense2.intrinsics()
        intrinsics.width = camera_info.width
        intrinsics.height = camera_info.height
        intrinsics.ppx = camera_info.K[2]
        intrinsics.ppy = camera_info.K[5]
        intrinsics.fx = camera_info.K[0]
        intrinsics.fy = camera_info.K[4]
        intrinsics.model  = pyrealsense2.distortion.none
        intrinsics.coeffs = [i for i in camera_info.D]
        result = pyrealsense2.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
        return result[2] / 1000.0, -result[0] / 1000.0, -result[1] / 1000.0

    def _calc_landmark_pose(self, landmark, depth_image):
        row, col = depth_image.shape
        px = int(landmark.x * col)
        py = int(landmark.y * row)
        if px < 0 and px > col-1 and py < 0 and py < row-1:
            raise Exception("Invalid coordinate")

        depth = depth_image[py, px]
        if depth == 0:
            raise Exception("Invalid depth value")

        pos = self._convert_depth_to_coord(px, py, depth, self._camera_info)
        return pos


    def run(self):
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()

            # if renew messages
            if self._rgb_image is None or self._depth_image is None or self._camera_info is None:
                continue

            image = cv2.cvtColor(cv2.flip(self._rgb_image, 1), cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = self._pose.process(image)

            # check result
            if results.pose_landmarks is None:
                continue

            # resize depth
            depth_image = cv2.resize(cv2.flip(self._depth_image, 1), dsize=(self._rgb_image.shape[1], self._rgb_image.shape[0]))

            # https://google.github.io/mediapipe/solutions/pose.html
            # A list of pose landmarks. Each landmark consists of the following:
            # x and y: Landmark coordinates normalized to [0.0, 1.0] by the image width and height respectively.
            # z: Represents the landmark depth with the depth at the midpoint of hips being the origin, and the smaller the value the closer the landmark is to the camera. The magnitude of z uses roughly the same scale as x.
            # visibility: A value in [0.0, 1.0] indicating the likelihood of the landmark being visible (present and not occluded) in the image.
            rshoulder = results.pose_landmarks.landmark[11]
            lshoulder = results.pose_landmarks.landmark[12]
            origin = None
            if min(rshoulder.visibility, lshoulder.visibility)  > 0.8:
                try:
                    rpose = np.array(self._calc_landmark_pose(rshoulder, depth_image))
                    lpose = np.array(self._calc_landmark_pose(lshoulder, depth_image))
                    origin = (rpose + lpose) / 2.0
                except Exception as e:
                    rospy.logwarn(e)

            rwrist = results.pose_landmarks.landmark[15]
            rthumb = results.pose_landmarks.landmark[21]
            if rwrist.visibility > 0.8 and origin is not None:
                try:
                    pose = np.array(self._calc_landmark_pose(rwrist, depth_image)) - origin

                    # publish pose data
                    self._rhand_pose.header.stamp = rospy.Time.now()
                    self._rhand_pose.pose.position = geometry_msgs.msg.Point(-pose[0], pose[1], pose[2])
                    self._rhand_pose_pub.publish(self._rhand_pose)
                except Exception as e:
                    rospy.logwarn(e)

            lwrist = results.pose_landmarks.landmark[16]
            lthumb = results.pose_landmarks.landmark[22]
            if lwrist.visibility > 0.8 and origin is not None:
                try:
                    pose = np.array(self._calc_landmark_pose(lwrist, depth_image)) - origin

                    # publish pose data
                    self._lhand_pose.header.stamp = rospy.Time.now()
                    self._lhand_pose.pose.position = geometry_msgs.msg.Point(-pose[0], pose[1], pose[2])
                    self._lhand_pose_pub.publish(self._lhand_pose)
                except Exception as e:
                    rospy.logwarn(e)

            rshoulder = results.pose_landmarks.landmark[11]
            lshoulder = results.pose_landmarks.landmark[12]


            # draw landmark on raw image
            if self._publish_pose_image:
                depth_8bit_image = np.uint8(depth_image / 64)
                cv2.cvtColor(depth_8bit_image, cv2.COLOR_GRAY2RGB)
                depth_image = cv2.cvtColor(depth_8bit_image, cv2.COLOR_GRAY2RGB)
                mp_drawing.draw_landmarks(
                    depth_image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                mp_drawing.draw_landmarks(
                    image,
                    results.pose_landmarks,
                    mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

                try:
                    image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_RGB2BGR)
                    pub_image = self._bridge.cv2_to_imgmsg(image, "rgb8")
                    self._pose_image_pub.publish(pub_image)

                    depth_image = cv2.cvtColor(cv2.flip(depth_image, 1), cv2.COLOR_RGB2BGR)
                    pub_depth_image = self._bridge.cv2_to_imgmsg(depth_image, "rgb8")
                    self._pose_depth_image_pub.publish(pub_depth_image)
                except Exception as e:
                    rospy.logerr(e)


def main():
    node = Node()
    node.run()
    return 0

if __name__ == '__main__':
    main()
