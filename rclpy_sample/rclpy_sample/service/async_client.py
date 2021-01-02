from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class AsyncServiceClient(Node):

    def __init__(self):
        super().__init__('async_service_client')
        self.logger = self.get_logger()

        self._cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')
        self.future = None
        self.req = AddTwoInts.Request()

        self.logger.info('----- [action_client]Start! -----')

    def send_request(self):
        self.req.a = 41
        self.req.b = 1
        self.future = self._cli.call_async(self.req)
        self.logger.info('----- [service_client] Request! -----')


def main(args=None):

    rclpy.init(args=args)
    srv_cli = AsyncServiceClient()

    try:
        while rclpy.ok():
            rclpy.spin_once(srv_cli, timeout_sec=0.1)
            if srv_cli.future is None:
                srv_cli.send_request()
            else:
                if srv_cli.future.done():
                    rsp = srv_cli.future.result()
                    srv_cli.get_logger().info(
                        'Result of add_two_ints: for %d + %d = %d' %
                        (srv_cli.req.a, srv_cli.req.b, rsp.sum))
                    srv_cli.future = None
    except KeyboardInterrupt:
        pass

    srv_cli.destroy_node()
    rclpy.shutdown()
