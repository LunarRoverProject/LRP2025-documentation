#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimpleSubscriber(Node):
    """
    最もシンプルなROS2サブスクライバーの例
    """
    
    def __init__(self):
        super().__init__('simple_subscriber')
        
        # サブスクライバーの作成
        # トピック名: 'hello_topic', メッセージ型: String, キューサイズ: 10
        self.subscription = self.create_subscription(
            String,
            'hello_topic',
            self.listener_callback,
            10
        )
        
        self.get_logger().info('シンプルサブスクライバーが開始されました')
    
    def listener_callback(self, msg):
        """
        メッセージ受信時のコールバック関数
        """
        self.get_logger().info(f'受信: {msg.data}')


def main(args=None):
    """
    メイン関数
    """
    # ROS2の初期化
    rclpy.init(args=args)
    
    # ノードの作成
    simple_subscriber = SimpleSubscriber()
    
    try:
        # ノードを実行（Ctrl+Cで停止するまで）
        rclpy.spin(simple_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # ノードの破棄
        simple_subscriber.destroy_node()
        # ROS2の終了
        rclpy.shutdown()


if __name__ == '__main__':
    main() 