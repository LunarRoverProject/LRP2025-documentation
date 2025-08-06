#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PubAndSubNode(Node):
    """
    パブリッシャとサブスクライバを一つずつ持つノードの例
    """
    def __init__(self):
        super().__init__('pub_and_sub_node')
        self.publisher = self.create_publisher(String, 'pubsub_topic', 10)
        self.subscription = self.create_subscription(
            String, 'pubsub_topic', self.sub_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_msg)
        self.count = 0
        self.get_logger().info('パブリッシャ・サブスクライバノードが開始されました')

    def publish_msg(self):
        msg = String()
        msg.data = f'カウント: {self.count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'送信: {msg.data}')
        self.count += 1

    def sub_callback(self, msg):
        self.get_logger().info(f'受信: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = PubAndSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()