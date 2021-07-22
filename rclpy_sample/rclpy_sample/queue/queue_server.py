import time
import datetime as dt
import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue


class QueueServer(Node):

    def __init__(self):
        super().__init__("queue_server")
        self.logger = self.get_logger()

        self._NAME_IDX = "idx"
        self._NAME_DATA = "data"

        cg_re_01 = ReentrantCallbackGroup()
        cg_mu_01 = MutuallyExclusiveCallbackGroup()

        self._srv = self.create_service(GetParameters,
                                        "get_parameters",
                                        callback=self.get_parameters_callback,
                                        callback_group=cg_re_01
                                        )

        self._start_dt = dt.datetime.now()
        self._df = pd.DataFrame(columns=[self._NAME_IDX, self._NAME_DATA])
        self._df.set_index(self._NAME_IDX, inplace=True)
        self._idx = 0
        self.logger.info("----- [queue_server]Start! -----")

    async def get_parameters_callback(self, req, rsp):
        dtstr = self._get_gap_str_datetime()
        self.logger.info("[{}]Service start! ==========".format(dtstr))

        # 0.5秒待機（重たい処理の代わり）
        length = len(self._df)
        for i, row in self._df.iterrows():
            self.logger.info("<Srv>[{}/{}]{}".format(i, length, row.loc[self._NAME_DATA]))
            para = ParameterValue()
            para.string_value = row.loc[self._NAME_DATA]
            time.sleep(0.5)
        rsp.values.append(para)

        dtstr = self._get_gap_str_datetime()
        self.logger.info("[{}]Service end! ==========".format(dtstr))

        return rsp

    def do_timeout_event(self) -> None:
        dtstr = self._get_gap_str_datetime()
        self._idx += 1
        series = pd.Series(data=dtstr, index=[self._NAME_DATA], name=self._idx)
        self._df = self._df.append(series)
        self.logger.info("----- do_timeout_event <Last_data:[{}]> -----"
                         .format(dtstr))

    def destroy(self):
        self._srv.destroy()
        super().destroy_node()

    def _get_gap_str_datetime(self):
        return str(dt.datetime.now() - self._start_dt)


def main(args=None):

    rclpy.init(args=args)
    que_srv = QueueServer()
    # マルチスレッドでasync_service_serverノードを実行し、
    # 複数のアクションクライアントを同時処理
    executor = MultiThreadedExecutor()
    # executor = SingleThreadedExecutor()

    try:
        while rclpy.ok():
            rclpy.spin_once(que_srv, executor=executor, timeout_sec=1.0)
            que_srv.do_timeout_event()
    except KeyboardInterrupt:
        pass

    que_srv.destroy()
    rclpy.shutdown()
