#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading
import time


class TwistMultiplexer(Node):
    def __init__(self):
        super().__init__('twist_multiplexer')

        # Prioritas: semakin atas, semakin tinggi prioritas
        self.sources = ['keyboard', 'joy', 'autonomous']
        self.topic_map = {
            'keyboard': 'keyboard_vel',
            'joy': 'joy_vel',
            'autonomous': 'autonomous_vel'
        }

        self.last_msg_time = {src: 0.0 for src in self.sources}
        self.latest_twist = {src: None for src in self.sources}
        self.lock = threading.Lock()
        self.timeout = 0.5  # detik

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd_type_pub = self.create_publisher(String, 'cmd_type', 10)

        # Subscriptions
        for source, topic in self.topic_map.items():
            self.create_subscription(Twist, topic, lambda msg, s=source: self.callback(msg, s), 10)

        # Timer untuk publish twist berdasarkan prioritas
        self.create_timer(0.05, self.publish_highest_priority_twist)

        self.get_logger().info('Multiplexer aktif. Prioritas: keyboard > joy > autonomous')

    def callback(self, msg, source):
        with self.lock:
            self.latest_twist[source] = msg
            self.last_msg_time[source] = time.time()

    def publish_highest_priority_twist(self):
        with self.lock:
            now = time.time()
            for source in self.sources:
                msg_time = self.last_msg_time[source]
                if self.latest_twist[source] is not None and now - msg_time < self.timeout:
                    self.cmd_vel_pub.publish(self.latest_twist[source])
                    cmd_type = String()
                    cmd_type.data = source
                    self.cmd_type_pub.publish(cmd_type)
                    return  # hanya publish satu, sesuai prioritas


def main(args=None):
    rclpy.init(args=args)
    node = TwistMultiplexer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
