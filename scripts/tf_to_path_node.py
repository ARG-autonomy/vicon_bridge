#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class TFPathPublisher:
    def __init__(self):
        rospy.init_node('tf_to_path_node')

        self.rate = rospy.Rate(10.0)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = rospy.get_param('~fixed_frame', 'map')

        self.source_frame = rospy.get_param('~source_frame', 'base_link')
        self.target_frame = rospy.get_param('~target_frame', 'map')

    def run(self):
        while not rospy.is_shutdown():
            try:
                now = rospy.Time.now()
                self.tf_buffer.can_transform(self.target_frame, self.source_frame, now, rospy.Duration(1.0))
                transform = self.tf_buffer.lookup_transform(self.target_frame,
                                                            self.source_frame,
                                                            now,
                                                            rospy.Duration(1.0))
                pose = PoseStamped()
                pose.header.stamp = transform.header.stamp
                pose.header.frame_id = self.target_frame
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation

                self.path_msg.header.stamp = rospy.Time.now()
                self.path_msg.poses.append(pose)

                self.path_pub.publish(self.path_msg)

            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(5.0, "TF lookup failed.")
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = TFPathPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
