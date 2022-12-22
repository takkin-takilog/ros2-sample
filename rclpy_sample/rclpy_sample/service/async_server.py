import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from example_interfaces.srv import AddTwoInts


class AsyncServiceServer(Node):
    """Async service-server node"""

    def __init__(self):
        super().__init__("async_service_server")
        self.logger = self.get_logger()

        self._mutua_cb_grp_01 = MutuallyExclusiveCallbackGroup()
        self._mutua_cb_grp_02 = MutuallyExclusiveCallbackGroup()
        self._mutua_cb_grp_03 = MutuallyExclusiveCallbackGroup()
        self._reent_cb_grp_01 = ReentrantCallbackGroup()

        # ----- Create service-server -----
        self._srv = self.create_service(
            AddTwoInts,
            "add_two_ints",
            callback=self._add_two_ints_callback,
            callback_group=self._reent_cb_grp_01,
        )

        # ----- Create timer -----
        self._timer01_cnt = 0
        self._timer01 = self.create_timer(
            3.0, callback=self._on_timer01, callback_group=self._mutua_cb_grp_02
        )
        self._timer02_cnt = 0
        self._timer02 = self.create_timer(
            3.0, callback=self._on_timer02, callback_group=self._mutua_cb_grp_03
        )

        self._counter = 0
        self._start_time = time.time()

        self.logger.info(
            "<{:.2f}>[service-server] Start".format(self._get_elapsed_time())
        )

    def _add_two_ints_callback(self, req, rsp):
        self.logger.info(
            "<{:.2f}>[callback-func] Start(a={}, b={})".format(
                self._get_elapsed_time(), req.a, req.b
            )
        )

        rsp.sum = req.a + req.b

        # 2 sec wait(instead of heavy process)
        time.sleep(2)

        self.logger.info(
            "<{:.2f}>[callback-func] End(a={}, b={})".format(
                self._get_elapsed_time(), req.a, req.b
            )
        )
        self._counter += 1

        return rsp

    def _on_timer01(self) -> None:
        self.logger.info("<{:.2f}>[timer01] Start".format(self._get_elapsed_time()))

        # 1 sec wait(instead of heavy process)
        time.sleep(1)

        self.logger.info("<{:.2f}>[timer01] End".format(self._get_elapsed_time()))

    def _on_timer02(self) -> None:
        self.logger.info("<{:.2f}>[timer02] Start".format(self._get_elapsed_time()))

        # 1 sec wait(instead of heavy process)
        time.sleep(1)

        self.logger.info("<{:.2f}>[timer02] End".format(self._get_elapsed_time()))

    def _get_elapsed_time(self):
        return time.time() - self._start_time


def main(args=None):

    rclpy.init(args=args)
    srv_srv = AsyncServiceServer()

    executor = MultiThreadedExecutor()
    # executor = SingleThreadedExecutor()

    try:
        rclpy.spin(srv_srv, executor)
    except KeyboardInterrupt:
        pass

    srv_srv.destroy_node()
    rclpy.shutdown()
