import time
import datetime as dt
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from example_interfaces.srv import AddTwoInts


class AsyncServiceServer(Node):

    def __init__(self):
        super().__init__("async_service_server")
        self.logger = self.get_logger()
        # add_two_intsサービスサーバーの作成
        # （execute_callback実行は複数同時処理を許可）
        cg_re_01 = ReentrantCallbackGroup()
        cg_mu_01 = MutuallyExclusiveCallbackGroup()

        self._srv = self.create_service(AddTwoInts,
                                        "add_two_ints",
                                        callback=self.add_two_ints_callback,
                                        callback_group=cg_re_01
                                        )

        # ---------- Timer 01 ----------
        self._timer01_cnt = 0
        self._timer01 = self.create_timer(timer_period_sec=3.0,
                                          callback=self._on_timer01,
                                          callback_group=cg_mu_01)

        # ---------- Timer 01 ----------
        self._timer02_cnt = 0
        self._timer02 = self.create_timer(timer_period_sec=3.0,
                                          callback=self._on_timer02,
                                          callback_group=cg_mu_01)

        self._dtstr = self._get_time_now()
        self.list_len = 3
        self._share_list = list(range(self._timer01_cnt, self._timer01_cnt + self.list_len))
        self.logger.info('----- [service_server]Start! -----')

    async def add_two_ints_callback(self, req, rsp):
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Service start! ==========".format(dtstr))

        # 1秒待機（重たい処理の代わり）
        alist = []
        for i, v in enumerate(self._share_list):
            self.logger.info("<Srv>[{}]{} ({})".format(i, v, self._share_list))
            alist.append(v)
            time.sleep(1)
        self.logger.info("<Srv>List:{}".format(alist))

        rsp.sum = req.a + req.b

        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Service end! ==========".format(dtstr))

        return rsp

    def _on_timer01(self) -> None:
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Timer01_in[{}] -----".format(dtstr, self._timer01_cnt))

        self._share_list = list(range(self._timer01_cnt, self._timer01_cnt + self.list_len))
        self.logger.info("<Timer>share_list:{}".format(self._share_list))

        time.sleep(1)
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Timer01_out[{}] -----".format(dtstr, self._timer01_cnt))
        self._timer01_cnt += 1

    def _on_timer02(self) -> None:
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Timer02_in[{}] -----".format(dtstr, self._timer02_cnt))
        time.sleep(1)
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Timer02_out[{}] -----".format(dtstr, self._timer02_cnt))
        self._timer02_cnt += 1

    def destroy(self):
        self._srv.destroy()
        super().destroy_node()

    def _get_time_now(self):
        dtnow = dt.datetime.now()
        return dtnow


def main(args=None):

    rclpy.init(args=args)
    srv_srv = AsyncServiceServer()
    # マルチスレッドでasync_service_serverノードを実行し、
    # 複数のアクションクライアントを同時処理
    executor = MultiThreadedExecutor()
    # executor = SingleThreadedExecutor()

    try:
        rclpy.spin(srv_srv, executor=executor)
    except KeyboardInterrupt:
        pass

    srv_srv.destroy()
    rclpy.shutdown()

