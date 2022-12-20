import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from example_interfaces.srv import AddTwoInts


class AsyncServiceClient(Node):
    """Async service-client node"""

    def __init__(self):
        super().__init__("async_service_client")
        self.logger = self.get_logger()

        self._mutua_cb_grp_01 = MutuallyExclusiveCallbackGroup()
        self._mutua_cb_grp_02 = MutuallyExclusiveCallbackGroup()
        # self._reent_cb_grp_01 = ReentrantCallbackGroup()

        # ----- Create service-client -----
        self._cli = self.create_client(
            AddTwoInts, "add_two_ints", callback_group=self._mutua_cb_grp_01
        )
        while not self._cli.wait_for_service(timeout_sec=1.0):
            self.logger.info("service not available, waiting again...")
        self._future = None

        # ----- Create timer -----
        self._timer01 = self.create_timer(
            1.0, self._on_timeout, callback_group=self._mutua_cb_grp_02
        )

        self._init_a = 0
        self._counter = 0
        self._start_time = time.time()

        self.logger.info("[service-client] Start")

    def _send_request(self):
        req = AddTwoInts.Request()
        req.a = self._init_a
        req.b = 1
        self._future = self._cli.call_async(req)
        self._init_a += 1
        self.logger.info(
            "<{:.2f}>[service-client] Request(a={}, b={})".format(
                self._get_elapsed_time(), req.a, req.b
            )
        )

    def _on_timeout(self) -> None:
        self.logger.info(
            "<{:.2f}>[timeout](cnt={})".format(self._get_elapsed_time(), self._counter)
        )
        self._send_request()
        self._counter += 1

    def _get_elapsed_time(self):
        return time.time() - self._start_time


def main(args=None):

    rclpy.init(args=args)
    srv_cli = AsyncServiceClient()

    executor = MultiThreadedExecutor()
    # executor = SingleThreadedExecutor()

    try:
        rclpy.spin(srv_cli, executor)
    except KeyboardInterrupt:
        pass

    srv_cli.destroy_node()
    rclpy.shutdown()
