import time
import datetime as dt
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from example_interfaces.action import Fibonacci

"""
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] sequence
"""


class AsyncActionServer(Node):

    def __init__(self):
        super().__init__('async_action_server')
        self.logger = self.get_logger()
        # fibonacciアクションサーバーの作成
        # （execute_callback実行は複数同時処理を許可）
        cg01 = ReentrantCallbackGroup()
        # cg01 = MutuallyExclusiveCallbackGroup()

        self._srv = ActionServer(self, Fibonacci, 'fibonacci',
                                 execute_callback=self._execute_callback,
                                 callback_group=cg01
                                 )

        # ---------- Timer 01 ----------
        self._timer01_cnt = 0
        self._timer01 = self.create_timer(timer_period_sec=3.0,
                                          callback=self._on_timer01,
                                          callback_group=cg01)

        # ---------- Timer 01 ----------
        self._timer02_cnt = 0
        self._timer02 = self.create_timer(timer_period_sec=3.0,
                                          callback=self._on_timer02,
                                          callback_group=cg01)

        self._dtstr = self._get_time_now()
        self.logger.info('----- [action_server]Start! -----')

    async def _execute_callback(self, goal_handle):
        # アクションの実行
        self.get_logger().info('executing...')
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Action start!".format(dtstr))

        # フィボナッチ数列の初期値0, 1を設定
        msg = Fibonacci.Feedback()
        msg.sequence = [0, 1]

        # フィボナッチ数列を一つずつ追加
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                # アクションのキャンセル
                goal_handle.set_canceled()
                self.get_logger().info('goal canceled')
                return Fibonacci.Result()

            # フィボナッチ数列の更新
            msg.sequence.append(msg.sequence[i] + msg.sequence[i - 1])
            self.get_logger().info('feedback: {0}'.format(msg.sequence))
            # アクションのフィードバックの送信
            goal_handle.publish_feedback(msg)
            # 1秒待機（重たい処理の代わり）
            time.sleep(1)

        # アクションの実行結果の送信
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = msg.sequence
        self.get_logger().info('result: {0}'.format(result.sequence))
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Action end!".format(dtstr))
        return result

    def _on_timer01(self) -> None:
        dtstr = self._get_time_now() - self._dtstr
        self.logger.info("[{}]Timer01_in[{}] -----".format(dtstr, self._timer01_cnt))
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
        # アクションサーバーの終了
        self._srv.destroy()
        super().destroy_node()

    def _get_time_now(self):
        dtnow = dt.datetime.now()
        return dtnow


def main(args=None):

    rclpy.init(args=args)
    act_srv = AsyncActionServer()
    # マルチスレッドでasync_action_serverノードを実行し、
    # 複数のアクションクライアントを同時処理
    executor = MultiThreadedExecutor()
    # executor = SingleThreadedExecutor()

    try:
        rclpy.spin(act_srv, executor=executor)
    except KeyboardInterrupt:
        pass

    act_srv.destroy()
    rclpy.shutdown()
