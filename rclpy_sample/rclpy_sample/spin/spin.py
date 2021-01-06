import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String


class SpinTest(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(String,
                                            "request",
                                            self.sub_callback,
                                            qos_profile)

        self.pub = self.create_publisher(String,
                                         "respons",
                                         qos_profile)

        self.counter = 0

    def sub_callback(self, msg_sub):
        msg_pub = String()
        msg_pub.data = "Hello World!"
        self.pub.publish(msg_pub)
        self.get_logger().info("[{}]{}".format(self.counter, msg_sub.data))
        self.counter += 1


def main_spin(args=None):
    rclpy.init(args=args)
    spin_test = SpinTest("spin_test")
    spin_test.get_logger().info("---------- [main_spin]Start! ----------")

    try:
        rclpy.spin(spin_test)
    except KeyboardInterrupt:
        pass

    spin_test.destroy_node()
    rclpy.shutdown()


def main_spin_once(args=None):
    rclpy.init(args=args)
    spin_test = SpinTest("spin_once_test")
    spin_test.get_logger().info("---------- [main_spin_once]Start! ----------")

    try:
        while rclpy.ok():
            rclpy.spin_once(spin_test, timeout_sec=1.0)
            spin_test.get_logger().info("spin_once timeout!")
    except KeyboardInterrupt:
        pass

    spin_test.destroy_node()
    rclpy.shutdown()
