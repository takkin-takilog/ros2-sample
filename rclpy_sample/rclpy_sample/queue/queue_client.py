from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node


class QueueClient(Node):

    def __init__(self):
        super().__init__("queue_client")
        self.logger = self.get_logger()

        self._cli = self.create_client(GetParameters, "get_parameters")
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.logger.info("service not available, waiting again...")
        self.future = None
        self.req = GetParameters.Request()
        self.logger.info("----- [queue_client]Start! -----")

    def send_request(self):
        self.future = self._cli.call_async(self.req)
        self.logger.info("----- [queue_client] Request! -----")


def main(args=None):

    rclpy.init(args=args)
    que_cli = QueueClient()

    try:
        while rclpy.ok():
            rclpy.spin_once(que_cli, timeout_sec=1.0)
            if que_cli.future is None:
                que_cli.send_request()
            else:
                if que_cli.future.done():
                    rsp = que_cli.future.result()
                    que_cli.get_logger().info("Get Last Data: [{}]"
                                              .format(rsp.values[-1].string_value))
                    que_cli.future = None

    except KeyboardInterrupt:
        pass

    que_cli.destroy_node()
    rclpy.shutdown()
