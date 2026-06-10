#!/usr/bin/env python3
"""
Publish odom -> base_link TF from /odom while filtering duplicate timestamps.

Gazebo's diff-drive plugin can occasionally emit the same odometry transform
stamp more than once. tf2 warns loudly about those repeated samples, so this
node owns the TF publication and drops non-increasing stamps.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry


class OdomTfBroadcaster:
    def __init__(self):
        self._parent_frame = rospy.get_param("~parent_frame", "odom")
        self._child_frame = rospy.get_param("~child_frame", "base_link")
        self._last_stamp = rospy.Time(0)
        self._broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=20)

    def _odom_cb(self, msg: Odometry):
        stamp = msg.header.stamp
        if stamp == rospy.Time(0):
            stamp = rospy.Time.now()
        if stamp <= self._last_stamp:
            rospy.logwarn_throttle(
                5.0,
                "[odom_tf] skipped duplicate/non-increasing odom stamp %.6f",
                stamp.to_sec(),
            )
            return

        parent = msg.header.frame_id or self._parent_frame
        child = msg.child_frame_id or self._child_frame

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        self._broadcaster.sendTransform(tf_msg)
        self._last_stamp = stamp


def main():
    rospy.init_node("odom_tf_broadcaster", anonymous=False)
    OdomTfBroadcaster()
    rospy.loginfo("[odom_tf] publishing filtered odom -> base_link TF from /odom")
    rospy.spin()


if __name__ == "__main__":
    main()
