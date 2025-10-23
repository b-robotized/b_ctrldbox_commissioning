#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    qos_profile_system_default
)
from std_msgs.msg import String
import sys
import argparse
import time

class ListenerString(Node):
    """
    Listens for a string on a designated topic, prints the received string ONCE.
    Matches QoS to the target topic automatically.
    """
    def __init__(self, topic):
        super().__init__('listener_string')

        # match QoS if the topic exists, or use default if not
        qos_profile = qos_profile_system_default
        publishers_info = self.get_publishers_info_by_topic(topic)
        if publishers_info:
            template_profile = publishers_info[0].qos_profile
            qos_profile = QoSProfile(
                depth=10,
                reliability=template_profile.reliability,
                durability=template_profile.durability
            )

        self.get_logger().info(f"Waiting for message on topic {topic}.")

        self.sub = self.create_subscription(
            String,
            topic,
            self.callback,
            qos_profile
        )
        self.payload = None

    def callback(self, msg):
        self.payload = msg.data
        self.get_logger().info(f"Message received: {msg.data}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', required=True)
    parser.add_argument('--timeout', type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()

    listener = ListenerString(args.topic)

    start_time = time.time()
    while time.time() - start_time < args.timeout:
        rclpy.spin_once(listener, timeout_sec=0.1)
        if listener.payload is not None:
            print(listener.payload)
            sys.exit(0) # Exit code 0 - success

    print(f"ERROR: Timeout waiting for message on {args.topic}", file=sys.stderr)
    sys.exit(1) # Exit code 1 - failure
