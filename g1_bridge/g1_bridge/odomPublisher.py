import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from unitree_go.msg import SportModeState
import numpy as np
from scipy.spatial.transform import Rotation


# Offset from base_link to pelvis in the body frame (from URDF floating_base_joint)
PELVIS_Z_OFFSET = 0.78


class G1OdomBridge(Node):
    def __init__(self):
        super().__init__('g1_odom_bridge')

        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        self.sub = self.create_subscription(
            SportModeState, '/odommodestate', self.callback, qos
        )
        self.odom_pub = self.create_publisher(Odometry, '/leg_odom', 10)

        self.get_logger().info('G1 Odom Bridge started')

    def callback(self, msg: SportModeState):
        now = self.get_clock().now().to_msg()

        # Unitree quaternion: [w, x, y, z]
        quat_wxyz = msg.imu_state.quaternion
        # ROS convention: [x, y, z, w]
        qx, qy, qz, qw = float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3]), float(quat_wxyz[0])

        # Rotation from body (pelvis) to odom
        R = Rotation.from_quat([qx, qy, qz, qw])

        # Pelvis position in odom frame (from SportModeState)
        p_pelvis = np.array(msg.position)

        # base_link is 0.78m below pelvis in the body frame
        # p_base_link = p_pelvis + R * [0, 0, -PELVIS_Z_OFFSET]
        offset_body = np.array([0.0, 0.0, -PELVIS_Z_OFFSET])
        p_base_link = p_pelvis + R.apply(offset_body)

        # Velocity correction: v_base = v_pelvis + omega x offset
        omega = np.array([
            msg.imu_state.gyroscope[0],
            msg.imu_state.gyroscope[1],
            msg.imu_state.gyroscope[2],
        ])
        v_pelvis = np.array(msg.velocity)
        v_base_link = v_pelvis + np.cross(omega, offset_body)

        # Publish nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = p_base_link[0]
        odom.pose.pose.position.y = p_base_link[1]
        odom.pose.pose.position.z = p_base_link[2]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity in body frame (child_frame_id = base_link)
        odom.twist.twist.linear.x = float(v_base_link[0])
        odom.twist.twist.linear.y = float(v_base_link[1])
        odom.twist.twist.linear.z = float(v_base_link[2])
        odom.twist.twist.angular.x = float(omega[0])
        odom.twist.twist.angular.y = float(omega[1])
        odom.twist.twist.angular.z = msg.yaw_speed

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = G1OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
