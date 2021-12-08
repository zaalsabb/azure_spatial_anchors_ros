#!/usr/bin/env python  
import rospy
import numpy as np
import tf
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import Transform, PoseStamped, TransformStamped, Point, Quaternion
from std_srvs.srv import Empty
from std_msgs.msg import String
from asa_ros_msgs.msg import *
from asa_ros_msgs.srv import *
from nav_msgs.msg import Path
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
import cv_bridge

class Node:

    def __init__(self):

        rospy.init_node('asa_path')

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        self.buffer_rate = 10.0
        self.buffer_size = 100
        self.images_buffer = []
        self.caminfo = None
        self.path = None

        rospy.Subscriber('path', Path, self.callback1)
        rospy.Subscriber('image', Image, self.callback2)
        # rospy.Subscriber('image', CompressedImage, self.callback2)
        rospy.Subscriber('camera_info', CameraInfo, self.callback3)

        self.pub1 = rospy.Publisher('asa/image', Image, queue_size=1)
        self.pub2 = rospy.Publisher('asa/camera_info', CameraInfo, queue_size=1)

        while not rospy.is_shutdown():
            self.main_loop()
            rospy.sleep(0.01)

        rospy.spin()

    def main_loop(self):
        if len(self.images_buffer) == 0:
            return
        if self.caminfo is None:
            return
        # img_compressed = self.images_buffer[-1]

        # dt = abs(ps.header.stamp.to_sec() - img_compressed.header.stamp.to_sec())

        abandoned_images = []
        for img_compressed in self.images_buffer:
            ps = self.interpolate_pose(self.path, img_compressed.header.stamp)
            if ps is None:
                abandoned_images.append(img_compressed)
                continue
            cv_bridge_ = cv_bridge.CvBridge()
            # img_cv2 = cv_bridge_.compressed_imgmsg_to_cv2(img_compressed, desired_encoding="passthrough")
            # img = cv_bridge_.cv2_to_imgmsg(img_cv2, encoding="mono8")
            img = img_compressed
            img.header.stamp = ps.header.stamp
            img.header.frame_id = 'asa_camera'
            self.pub1.publish(img)

            self.br.sendTransform((ps.pose.position.x, ps.pose.position.y, ps.pose.position.z),
                                  (ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w),
                                  ps.header.stamp,
                                  'asa_baselink',
                                  ps.header.frame_id)
             
            self.br.sendTransform((0.15109427, -0.00254265, -0.03152154),
                                  (0.51484072, -0.5054356 ,  0.49331617, -0.48591465),
                                  ps.header.stamp,
                                  'asa_camera',
                                  'asa_baselink')         
            
            self.caminfo.header.stamp = ps.header.stamp
            self.pub2.publish(self.caminfo)
            # self.images_buffer.pop(self.images_buffer.index(img_compressed))
        
        self.images_buffer = abandoned_images        

    def callback1(self,msg):
        self.path = msg

    def callback2(self,msg):
        if len(self.images_buffer) > 0:
            if msg.header.stamp.to_sec() - self.images_buffer[-1].header.stamp.to_sec() < 1/self.buffer_rate:
                return
        self.images_buffer.append(msg)
        if len(self.images_buffer) >= self.buffer_size:
            self.images_buffer.pop(0)

    def callback3(self,msg):
        self.caminfo = msg

    def interpolate_pose(self, path, stamp):
        if path is None or len(path.poses) == 0:
            return
        t = stamp.to_sec()
        ps = PoseStamped()
        ps.header.frame_id = path.header.frame_id
        ps.header.stamp = stamp
        for i in range(len(path.poses)-1):
            pose1 = path.poses[i]
            pose2 = path.poses[i+1]
            t1 = pose1.header.stamp.to_sec()
            t2 = pose2.header.stamp.to_sec()
            if t >= t1 and t <= t2:
                dt = t2 - t1
                w = (t - t1) / dt
                ps.pose.position.x = pose1.pose.position.x + (pose2.pose.position.x - pose1.pose.position.x) * w
                ps.pose.position.y = pose1.pose.position.y + (pose2.pose.position.y - pose1.pose.position.y) * w
                ps.pose.position.z = pose1.pose.position.z + (pose2.pose.position.z - pose1.pose.position.z) * w
                q1 = np.array([pose1.pose.orientation.x,pose1.pose.orientation.y,pose1.pose.orientation.z,pose1.pose.orientation.w])
                q2 = np.array([pose2.pose.orientation.x,pose2.pose.orientation.y,pose2.pose.orientation.z,pose2.pose.orientation.w])
                q = tf.transformations.quaternion_slerp(q1, q2, w)
                ps.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])

                return ps
        return

if __name__ == '__main__':
    Node()