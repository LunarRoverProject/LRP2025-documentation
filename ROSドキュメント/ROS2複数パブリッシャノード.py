#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class MultiPublisherNode(Node):
    """
    複数のパブリッシャを持つノードの例
    """
    def __init__(self):
        super().__init__('multi_publisher_node')
        # 2つの異なるトピックに対するパブリッシャ
        self.string_pub = self.create_publisher(String, 'string_topic', 10)
        self.int_pub = self.create_publisher(Int32, 'int_topic', 10)
        # それぞれのタイマー
        self.string_timer = self.create_timer(1.0, self.publish_string)
        self.int_timer = self.create_timer(2.0, self.publish_int)
        self.int_count = 0
        self.get_logger().info('複数パブリッシャノードが開始されました')

    def publish_string(self):
        msg = String()
        msg.data = 'こんにちは from string_topic'
        self.string_pub.publish(msg)
        self.get_logger().info(f'string_topicに送信: {msg.data}')

    def publish_int(self):
        msg = Int32()
        msg.data = self.int_count
        self.int_pub.publish(msg)
        self.get_logger().info(f'int_topicに送信: {msg.data}')
        self.int_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MultiPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()