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

        cb_grp_ree_01 = ReentrantCallbackGroup()

        # Set logger lebel
        self.logger = super().get_logger()
        self.logger.set_level(rclpy.logging.LoggingSeverity.INFO)

        self.sub = self.create_subscription(msg_type=String,
                                            topic='chatter',
                                            callback=self.tpc_callback,
                                            callback_group=cb_grp_ree_01)

        # String型のchatterトピックを送信するpublisherの定義
        self.pub = self.create_publisher(String, 'callback')

        self.srv01 = self.create_service(srv_type=AddTwoInts,
                                         srv_name="add_two_ints_01",
                                         callback=self.srv_callback01,
                                         callback_group=cb_grp_ree_01)
        self.srv02 = self.create_service(srv_type=AddTwoInts,
                                         srv_name="add_two_ints_02",
                                         callback=self.srv_callback02,
                                         callback_group=cb_grp_ree_01)
        self.srv03 = self.create_service(srv_type=AddTwoInts,
                                         srv_name="add_two_ints_03",
                                         callback=self.srv_callback03,
                                         callback_group=cb_grp_ree_01)

    def get_time_now(self):
        dtnow = dt.datetime.now()
        return dtnow.strftime(Reciever.DT_FMT)

    def srv_callback01(self, request, response):

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Service01(%d): start" % (dtstr, request.a))

        sleep(1)

        response.sum = request.a

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Service01(%d): End" % (dtstr, request.a))

        return response

    def srv_callback02(self, request, response):

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Service02(%d): start" % (dtstr, request.a))

        sleep(1)
        response.sum = request.a

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Service02(%d): End" % (dtstr, request.a))

        return response

    def srv_callback03(self, request, response):

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Service03(%d): start" % (dtstr, request.a))

        sleep(1)
        response.sum = request.a

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Service03(%d): End" % (dtstr, request.a))

        return response

    def send_topic(self, arg):
        msg = String()
        msg.data = arg
        # chatterトピックにmsgを送信
        self.pub.publish(msg)

    def tpc_callback(self, msg):

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Topic    (%s): start" % (dtstr, msg.data))

        sleep(1)

        self.send_topic(msg.data)

        dtstr = self.get_time_now()
        self.logger.info("[%s]:Topic    (%s): end" % (dtstr, msg.data))


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

    #rclpy.spin(rcv)
    rclpy.shutdown()
