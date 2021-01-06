import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.constants import S_TO_NS
from std_msgs.msg import Float32
import datetime as dt


class SpinTest(Node):

    def __init__(self, node_name):
        super().__init__(node_name)

        qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_ALL,
                                 reliability=QoSReliabilityPolicy.RELIABLE)

        self.sub = self.create_subscription(Float32,
                                            "timer",
                                            self.sub_callback,
                                            qos_profile)

        self.counter = 0

        # Timer
        self.timer = self.create_timer(timer_period_sec=1.0,
                                       callback=self.on_timeout)
        self.start_time = dt.datetime.now()

    def sub_callback(self, msg_sub):
        self.get_logger().info("change timer[{}]->[{}]".format(self.timer.timer_period_ns / S_TO_NS, msg_sub.data))
        self.timer.timer_period_ns = int(float(msg_sub.data) * S_TO_NS)

    def on_timeout(self) -> None:
        time = dt.datetime.now()
        self.get_logger().info("Timeout![{}][{}]".format(self.counter, time - self.start_time))
        self.start_time = time
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    spin_test = SpinTest("timer")
    spin_test.get_logger().info("---------- [timer]Start! ----------")

    try:
        rclpy.spin(spin_test)
    except KeyboardInterrupt:
        pass

    spin_test.destroy_node()
    rclpy.shutdown()
