#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped

def callback(data):
    pose = PoseStamped()
    pose.header = data.header
    pose.pose.position.x = data.transform.translation.x
    pose.pose.position.y = data.transform.translation.y
    pose.pose.position.z = data.transform.translation.z
    pose.pose.orientation = data.transform.rotation
    pub.publish(pose)

rospy.init_node('vicon_to_mavros')
pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)
rospy.Subscriber('/vicon/UAV1/UAV1', TransformStamped, callback)
rospy.spin()
