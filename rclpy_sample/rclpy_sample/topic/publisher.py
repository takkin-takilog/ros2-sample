import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")

        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        # String型のchatterトピックを送信するpublisherの定義
        self.publisher = self.create_publisher(String,
                                               "chatter",
                                               qos_profile)
        # 送信周期毎にtimer_callbackを呼び出し（送信周期は0.5秒）
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = "Hello World!"
        # chatterトピックにmsgを送信
        self.publisher.publish(msg)
        # msgの中身を標準出力にログ
        self.get_logger().info(msg.data)


def main(args=None):
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    # minimal_publisherノードの作成
    minimal_publisher = MinimalPublisher()

    try:
        # minimal_publisherノードの実行開始
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass

    # ノードの破棄
    minimal_publisher.destroy_node()
    # Pythonクライアントライブラリの終了
    rclpy.shutdown()


if __name__ == "__main__":
    main()
