#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
import time

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg

def main():
    rospy.init_node('offboard_takeoff_land')

    # 发布位置 setpoint
    local_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # 订阅飞控状态
    rospy.Subscriber("/mavros/state", State, state_cb)

    rate = rospy.Rate(20)

    # 等待连接
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo_throttle(2.0, "Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("FCU connected")

    # 创建服务客户端
    rospy.wait_for_service("/mavros/cmd/arming")
    rospy.wait_for_service("/mavros/set_mode")
    arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    # 初始化目标位姿
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1.0  # 起飞目标高度

    # 预热 setpoint
    for _ in range(100):
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    # 切换模式为 OFFBOARD
    rospy.loginfo("Setting OFFBOARD mode...")
    set_mode_client(base_mode=0, custom_mode="OFFBOARD")

    # 解锁
    rospy.loginfo("Arming drone...")
    arming_client(True)

    rospy.loginfo("Taking off to 1.0 m...")
    takeoff_start = rospy.Time.now()

    # 起飞并保持一定时间
    while not rospy.is_shutdown() and (rospy.Time.now() - takeoff_start < rospy.Duration(10.0)):
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Landing...")

    # 修改目标高度为地面，触发位置式降落
    pose.pose.position.z = 0.0
    land_start = rospy.Time.now()

    while not rospy.is_shutdown() and (rospy.Time.now() - land_start < rospy.Duration(10.0)):
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Disarming...")
    arming_client(False)

    rospy.loginfo("Mission completed.")

if __name__ == "__main__":
    main()
