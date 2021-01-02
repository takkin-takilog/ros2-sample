import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
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


class AsyncActionClient(Node):

    def __init__(self):
        super().__init__('async_action_client')
        # fibonacciアクションクライアントを作成
        self._cli = ActionClient(self, Fibonacci, 'fibonacci')
        self.logger = self.get_logger()
        self.logger.info('----- [action_client]Start! -----')

    def send_goal(self):
        self.logger.info('waiting...')

        while not self._cli.wait_for_server(timeout_sec=1.0):
            self.logger.info("Waiting for \"fibonacci\" server...")

        # 10要素のフィボナッチ数列を標値に設定
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        # アクションの非同期実行
        # （フィードバックと実行結果の受信コールバック関数も設定）

        goal = goal_msg
        feedback_callback = self._feedback_callback
        self._goal_future = self._cli.send_goal_async(goal,
                                                      feedback_callback)
        callback = self._goal_response_callback
        self._goal_future.add_done_callback(callback)

    def _goal_response_callback(self, future):
        # 目標値の設定成功の判別
        send_gol_rsp = future.result()
        if not send_gol_rsp.accepted:
            self.logger.info('goal rejected')
            return

        # アクションの実行結果の受信
        self._result_future = send_gol_rsp.get_result_async()

        callback = self._get_result_callback
        self._result_future.add_done_callback(callback)

    def _feedback_callback(self, msg):

        self.logger.info('feedback: {0}'.format(msg.feedback.sequence))

    def _get_result_callback(self, future):
        # アクションの実行状態の取得
        rsp = future.result()
        status = rsp.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            # 実行成功ならフィボナッチ数列を標準出力にログ
            self.get_logger().info('result: {0}'.format(rsp.result.sequence))


def main(args=None):

    rclpy.init(args=args)
    act_cli = AsyncActionClient()

    try:
        act_cli.send_goal()
        rclpy.spin(act_cli)
    except KeyboardInterrupt:
        pass

    act_cli.destroy_node()
    rclpy.shutdown()
