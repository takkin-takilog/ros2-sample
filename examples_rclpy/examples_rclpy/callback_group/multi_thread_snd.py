import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import datetime as dt
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from time import sleep

class Sender(Node):

    DT_FMT = "%S.%f"

    def __init__(self):
        super().__init__("multi_thread_sender")

        # Set logger lebel
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.INFO)

        # ========== マルチスレッドテスト１ ==========
        self.cb_grp_ms_01_itr = 0
        cb_grp_ms_01 = ReentrantCallbackGroup()
        # Topic Send
        self.pub_grp01_ms = self.create_publisher(String, "tpc_grp01_ms_snd")
        # Topic Recieve
        msg_type = String
        topic = "tpc_grp01_ms_rcv"
        callback = self.on_tpc_grp01_ms_rcv
        self.sub_grp01_ms = self.create_subscription(msg_type=msg_type,
                                                     topic=topic,
                                                     callback=callback,
                                                     callback_group=cb_grp_ms_01)

        # Timer
        self.__timer_ms_grp01 = self.create_timer(timer_period_sec=3.0,
                                                  callback=self.__on_timeout_ms_grp01,
                                                  callback_group=cb_grp_ms_01)

        # ========== マルチスレッドテスト２ ==========
        self.cb_grp_ms_02_itr = 0
        cb_grp_ms_02 = MutuallyExclusiveCallbackGroup()
        # Timer
        self.__timer_ms_grp02 = self.create_timer(timer_period_sec=3.0,
                                                  callback=self.__on_timeout_ms_grp02,
                                                  callback_group=cb_grp_ms_02)


        self.cli01 = self.create_client(AddTwoInts, "add_two_ints_01")
        while not self.cli01.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.cli02 = self.create_client(AddTwoInts, "add_two_ints_02")
        while not self.cli02.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.cli03 = self.create_client(AddTwoInts, "add_two_ints_03")
        while not self.cli03.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.future01 = None
        self.future02 = None
        self.future03 = None

        self.tpc_seq_itr = 0
        self.srv_seq_itr = 0

        self.__dtstr = self.__get_time_now()

    def __on_timeout_ms_grp01(self) -> None:
        self.cb_grp_ms_01_itr = self.cb_grp_ms_01_itr + 1
        msg = String()
        msg.data = str(self.cb_grp_ms_01_itr)

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("----- [%s]:[ms_grp01]TimeOut Start(%d) -----" % (dtstr, self.cb_grp_ms_01_itr))

        self.pub_grp01_ms.publish(msg)

        sleep(1)
        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("----- [%s]:[ms_grp01]TimeOut End(%d) -----" % (dtstr, self.cb_grp_ms_01_itr))

    def on_tpc_grp01_ms_rcv(self, msg):
        # msgの中身を標準出力にログ
        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("----- [%s]:[ms_grp01]TPC Rcv(%s) -----" % (dtstr, msg.data))

    def __on_timeout_ms_grp02(self) -> None:
        self.cb_grp_ms_02_itr = self.cb_grp_ms_02_itr + 1

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("----- [%s]:[ms_grp02]TimeOut Start(%d) -----" % (dtstr, self.cb_grp_ms_02_itr))

        self.__send_srv_request()
        sleep(1)

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("----- [%s]:[ms_grp02]TimeOut End(%d) -----" % (dtstr, self.cb_grp_ms_02_itr))

    def __send_srv_request(self):
        req = AddTwoInts.Request()

        self.srv_seq_itr = self.srv_seq_itr + 1

        req.a = self.srv_seq_itr
        req.b = 0
        self.future01 = self.cli01.call_async(req)
        self.future02 = self.cli02.call_async(req)
        self.future03 = self.cli03.call_async(req)

        dtstr = self.__get_time_now() - self.__dtstr
        self.logger.info("[%s]:Service Send(%d)" % (dtstr, self.srv_seq_itr))

    def background(self):

        if ((self.future01 is not None) and (self.future01.done())):
            rsp = self.future01.result()
            dtstr = self.__get_time_now() - self.__dtstr
            self.logger.info("[%s]:Service01 Rcv(%d)" % (dtstr, rsp.sum))
            self.future01 = None

        if ((self.future02 is not None) and (self.future02.done())):
            rsp = self.future02.result()
            dtstr = self.__get_time_now() - self.__dtstr
            self.logger.info("[%s]:Service02 Rcv(%d)" % (dtstr, rsp.sum))
            self.future02 = None

        if ((self.future03 is not None) and (self.future03.done())):
            rsp = self.future03.result()
            dtstr = self.__get_time_now() - self.__dtstr
            self.logger.info("[%s]:Service03 Rcv(%d)" % (dtstr, rsp.sum))
            self.future03 = None

    def __get_time_now(self):
        dtnow = dt.datetime.now()
        #return dtnow.strftime(self.DT_FMT)
        return dtnow


def main(args=None):
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    srv = Sender()
    executor.add_node(srv)
    while rclpy.ok():
        executor.spin_once()
        srv.background()
    rclpy.shutdown()
