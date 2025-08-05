#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class AutonomousBridge(Node):
    def __init__(self):
        super().__init__('autonomous_bridge')

        # Declare and get topic parameters (optional override)
        self.declare_parameter('input_topic', 'autonomous_vel')
        self.declare_parameter('output_cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('output_cmd_type_topic', 'cmd_type')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_cmd_vel_topic = self.get_parameter('output_cmd_vel_topic').get_parameter_value().string_value
        output_cmd_type_topic = self.get_parameter('output_cmd_type_topic').get_parameter_value().string_value

        # Publisher to cmd_vel (Twist)
        self.cmd_vel_pub = self.create_publisher(Twist, output_cmd_vel_topic, 10)

        # Publisher to cmd_type (String)
        self.cmd_type_pub = self.create_publisher(String, output_cmd_type_topic, 10)

        # Subscriber to autonomous_vel
        self.subscription = self.create_subscription(
            Twist,
            input_topic,
            self.autonomous_callback,
            10
        )

        self.get_logger().info(f'AutonomousBridge started:')
        self.get_logger().info(f'  Subscribing to: {input_topic}')
        self.get_logger().info(f'  Publishing to: {output_cmd_vel_topic} (Twist)')
        self.get_logger().info(f'  Publishing to: {output_cmd_type_topic} (String)')

    def autonomous_callback(self, msg):
        # Publish the Twist message directly
        self.cmd_vel_pub.publish(msg)

        # Always publish "autonomous" as the source
        cmd_type_msg = String()
        cmd_type_msg.data = 'autonomous'
        self.cmd_type_pub.publish(cmd_type_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()