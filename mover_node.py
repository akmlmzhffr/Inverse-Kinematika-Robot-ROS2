import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class InverseKinematicsNode(Node):
    # Inverse kinematics untuk robot differential drive.
    #
    # Input:
    #   /cmd_vel (geometry_msgs/Twist)
    #     linear.x  = v [m/s]
    #     angular.z = omega [rad/s]
    #
    # Output:
    #   /wheel_velocities (std_msgs/Float64MultiArray)
    #     data[0] = roda kiri  [rad/s]
    #     data[1] = roda kanan [rad/s]
    #
    # Persamaan:
    #   w_left  = (v - omega*L/2) / R
    #   w_right = (v + omega*L/2) / R

    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Sesuaikan dengan ukuran robot di simulator.
        self.declare_parameter('wheel_radius', 0.05)  # meter
        self.declare_parameter('wheel_base', 0.30)    # meter

        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )
        self.wheel_base = float(
            self.get_parameter('wheel_base').value
        )

        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius harus > 0.')
        if self.wheel_base <= 0.0:
            raise ValueError('wheel_base harus > 0.')

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocities',
            10
        )

        self.last_left = 0.0
        self.last_right = 0.0

        self.get_logger().info('Inverse Kinematics Node aktif.')
        self.get_logger().info(
            f'wheel_radius = {self.wheel_radius:.3f} m'
        )
        self.get_logger().info(
            f'wheel_base   = {self.wheel_base:.3f} m'
        )

    def cmd_vel_callback(self, msg: Twist):
        # Kecepatan robot
        v = msg.linear.x
        omega = msg.angular.z

        # Inverse kinematics differential drive
        left_wheel = (
            v - (omega * self.wheel_base / 2.0)
        ) / self.wheel_radius

        right_wheel = (
            v + (omega * self.wheel_base / 2.0)
        ) / self.wheel_radius

        # Urutan output: [kiri, kanan]
        output = Float64MultiArray()
        output.data = [left_wheel, right_wheel]
        self.wheel_pub.publish(output)

        # Logging hanya ketika perubahan cukup besar.
        if (
            abs(left_wheel - self.last_left) > 0.01
            or abs(right_wheel - self.last_right) > 0.01
        ):
            self.get_logger().info(
                f'v={v:.2f} m/s, omega={omega:.2f} rad/s | '
                f'left={left_wheel:.2f} rad/s, '
                f'right={right_wheel:.2f} rad/s'
            )
            self.last_left = left_wheel
            self.last_right = right_wheel


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Pastikan output roda menjadi nol saat node dihentikan.
        stop = Float64MultiArray()
        stop.data = [0.0, 0.0]
        node.wheel_pub.publish(stop)

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
