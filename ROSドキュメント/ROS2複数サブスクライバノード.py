#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

class MultiSubscriberNode(Node):
    """
    複数のサブスクライバを持つノードの例
    """
    def __init__(self):
        super().__init__('multi_subscriber_node')
        # 2つの異なるトピックに対するサブスクライバ
        self.string_sub = self.create_subscription(
            String, 'string_topic', self.string_callback, 10)
        self.int_sub = self.create_subscription(
            Int32, 'int_topic', self.int_callback, 10)
        self.get_logger().info('複数サブスクライバノードが開始されました')

    def string_callback(self, msg):
        self.get_logger().info(f'string_topicから受信: {msg.data}')

    def int_callback(self, msg):
        self.get_logger().info(f'int_topicから受信: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()