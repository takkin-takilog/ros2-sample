import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import datetime as dt
from time import sleep
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor


class Reciever(Node):

    DT_FMT = "%S.%f"

    def __init__(self):
        super().__init__('multi_thread_reciver')

        # Set logger lebel
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.INFO)

        # ========== マルチスレッドテスト１ ==========
        cb_grp_ms_01 = ReentrantCallbackGroup()
        # Topic Recieve
        msg_type = String
        topic = "tpc_grp01_ms_snd"
        callback = self.on_tpc_grp01_ms_rcv
        self.sub_grp01_ms = self.create_subscription(msg_type=msg_type,
                                                     topic=topic,
                                                     callback=callback,
                                                     callback_group=cb_grp_ms_01)
        # Topic Send
        self.pub_grp01_ms = self.create_publisher(String, "tpc_grp01_ms_rcv")




        # ========== マルチスレッドテスト２ ==========
        cb_grp_ree_01 = MutuallyExclusiveCallbackGroup()
        cb_grp_ree_02 = MutuallyExclusiveCallbackGroup()
        cb_grp_ree_03 = MutuallyExclusiveCallbackGroup()

        # String型のchatterトピックを送信するpublisherの定義
        self.pub = self.create_publisher(String, 'tpc_test_rcv')

        self.srv01 = self.create_service(srv_type=AddTwoInts,
                                         srv_name="add_two_ints_01",
                                         callback=self.srv_callback01,
                                         callback_group=cb_grp_ree_01)
        self.srv02 = self.create_service(srv_type=AddTwoInts,
                                         srv_name="add_two_ints_02",
                                         callback=self.srv_callback02,
                                         callback_group=cb_grp_ree_02)
        self.srv03 = self.create_service(srv_type=AddTwoInts,
                                         srv_name="add_two_ints_03",
                                         callback=self.srv_callback03,
                                         callback_group=cb_grp_ree_03)

        self.__dtstr = self.__get_time_now()

    def on_tpc_grp01_ms_rcv(self, msg):
        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("----- [%s]:[ms_grp01]TPC Rcv(%s) -----" % (dtstr, msg.data))
        self.pub_grp01_ms.publish(msg)

    def srv_callback01(self, request, response):

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("[%s]:Service01(%d): start" % (dtstr, request.a))

        sleep(1)

        response.sum = request.a

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("[%s]:Service01(%d): End" % (dtstr, request.a))

        return response

    def srv_callback02(self, request, response):

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("[%s]:Service02(%d): start" % (dtstr, request.a))

        sleep(1)
        response.sum = request.a

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("[%s]:Service02(%d): End" % (dtstr, request.a))

        return response

    def srv_callback03(self, request, response):

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("[%s]:Service03(%d): start" % (dtstr, request.a))

        sleep(1)
        response.sum = request.a

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("[%s]:Service03(%d): End" % (dtstr, request.a))

        return response

    def __get_time_now(self):
        dtnow = dt.datetime.now()
        return dtnow


def main(args=None):
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    rcv = Reciever()

    executor.add_node(rcv)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    rcv.destroy_node()
