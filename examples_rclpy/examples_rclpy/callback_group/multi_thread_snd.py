import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import AddTwoInts
import datetime as dt


class Sender(Node):

    DT_FMT = "%S.%f"

    def __init__(self):
        super().__init__('multi_thread_sender')

        # Set logger lebel
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.INFO)

        # String型のchatterトピックを送信するpublisherの定義
        self.pub = self.create_publisher(String, 'chatter')

        self.sub = self.create_subscription(String, 'callback', self.tpc_callback)

        self.cli01 = self.create_client(AddTwoInts, 'add_two_ints_01')
        while not self.cli01.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cli02 = self.create_client(AddTwoInts, 'add_two_ints_02')
        while not self.cli02.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.cli03 = self.create_client(AddTwoInts, 'add_two_ints_03')
        while not self.cli03.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.future01 = None
        self.future02 = None
        self.future03 = None

        # 送信周期毎にtimer_callbackを呼び出し（送信周期は0.5秒）
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.tpc_seq_itr = 0
        self.srv_seq_itr = 0

    def tpc_callback(self, msg):
        # msgの中身を標準出力にログ
        dtstr = self.get_time_now()
        self.logger.info("[%s]:Topic   Rcv (%s)" % (dtstr, msg.data))

    def send_topic(self):

        self.tpc_seq_itr = self.tpc_seq_itr + 1

        msg = String()
        msg.data = str(self.tpc_seq_itr)
        # chatterトピックにmsgを送信
        self.pub.publish(msg)

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Topic   Send(%d)" % (dtstr, self.tpc_seq_itr))

    def send_srv_request(self):
        req = AddTwoInts.Request()

        self.srv_seq_itr = self.srv_seq_itr + 1

        req.a = self.srv_seq_itr
        req.b = 0
        self.future01 = self.cli01.call_async(req)
        self.future02 = self.cli02.call_async(req)
        self.future03 = self.cli03.call_async(req)

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Service Send(%d)" % (dtstr, self.srv_seq_itr))

    def timer_callback(self):
        self.send_topic()
        self.send_srv_request()

    def get_time_now(self):
        dtnow = dt.datetime.now()
        return dtnow.strftime(Sender.DT_FMT)

    def background(self):

        if ((self.future01 is not None) and (self.future01.done())):
            rsp = self.future01.result()
            dtstr = self.get_time_now()
            self.logger.info("[%s]:Service01 Rcv(%d)" % (dtstr, rsp.sum))
            self.future01 = None

        if ((self.future02 is not None) and (self.future02.done())):
            rsp = self.future02.result()
            dtstr = self.get_time_now()
            self.logger.info("[%s]:Service02 Rcv(%d)" % (dtstr, rsp.sum))
            self.future02 = None

        if ((self.future03 is not None) and (self.future03.done())):
            rsp = self.future03.result()
            dtstr = self.get_time_now()
            self.logger.info("[%s]:Service03 Rcv(%d)" % (dtstr, rsp.sum))
            self.future03 = None


def main(args=None):
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    srv = Sender()
    while rclpy.ok():
        rclpy.spin_once(srv)
        srv.background()
    rclpy.shutdown()
