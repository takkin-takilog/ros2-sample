import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")

        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        # String型のchatterトピックを受信するsubscriptionの定義
        # （listener_callbackは受信毎に呼び出されるコールバック関数）
        self.subscription = self.create_subscription(String,
                                                     "chatter",
                                                     self.listener_callback,
                                                     qos_profile)

    def listener_callback(self, msg):
        # msgの中身を標準出力にログ
        self.get_logger().info(msg.data)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    minimal_subscriber.get_logger().info("minimal_subscriber start!")

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass

    """
    try:
        while rclpy.ok():
            rclpy.spin_once(minimal_subscriber, timeout_sec=10.0)
            minimal_subscriber.get_logger().info("spin_once timeout!")
    except KeyboardInterrupt:
        pass
    """

    minimal_subscriber.destroy_node()
    rclpy.shutdown()
