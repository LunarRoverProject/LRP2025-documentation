#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SimplePublisher(Node):
    """
    最もシンプルなROS2パブリッシャーの例
    """
    
    def __init__(self):
        super().__init__('simple_publisher')
        
        # パブリッシャーの作成
        # トピック名: 'hello_topic', メッセージ型: String, キューサイズ: 10
        self.publisher_ = self.create_publisher(String, 'hello_topic', 10)
        
        # タイマーの作成（1秒間隔でコールバック関数を実行）
        timer_period = 1.0  # 秒
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('シンプルパブリッシャーが開始されました')
    
    def timer_callback(self):
        """
        タイマーコールバック関数
        定期的にメッセージをパブリッシュする
        """
        msg = String()
        msg.data = f'Hello ROS2! 時刻: {self.get_clock().now().to_msg().sec}'
        
        # メッセージをパブリッシュ
        self.publisher_.publish(msg)
        self.get_logger().info(f'パブリッシュ: {msg.data}')


def main(args=None):
    """
    メイン関数
    """
    # ROS2の初期化
    rclpy.init(args=args)
    
    # ノードの作成
    simple_publisher = SimplePublisher()
    
    try:
        # ノードを実行（Ctrl+Cで停止するまで）
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # ノードの破棄
        simple_publisher.destroy_node()
        # ROS2の終了
        rclpy.shutdown()


if __name__ == '__main__':
    main()
