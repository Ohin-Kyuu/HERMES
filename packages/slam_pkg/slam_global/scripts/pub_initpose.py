#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import tf_transformations

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('publish_initial_pose')

    x     = node.declare_parameter('x', 0.0).value
    y     = node.declare_parameter('y', 0.0).value
    z     = node.declare_parameter('z', 0.0).value
    roll  = node.declare_parameter('roll', 0.0).value
    pitch = node.declare_parameter('pitch', 0.0).value
    yaw   = node.declare_parameter('yaw', 0.0).value

    quat_xyzw = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
    qx, qy, qz, qw = quat_xyzw

    p = Point()
    p.x = x
    p.y = y
    p.z = z

    q = Quaternion()
    q.x = qx
    q.y = qy
    q.z = qz
    q.w = qw

    pose_msg = Pose()
    pose_msg.position = p
    pose_msg.orientation = q

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose = pose_msg
    msg.pose.covariance = [0.0]*36

    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    pub.publish(msg)

    node.get_logger().info(
        f'Published initial pose (tf_transformations): '
        f'pos=({x:.2f}, {y:.2f}, {z:.2f}) '
        f'rpy=({roll:.2f}, {pitch:.2f}, {yaw:.2f})'
    )

    rclpy.shutdown()

if __name__ == '__main__':
    main()
