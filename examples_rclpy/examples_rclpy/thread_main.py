import rclpy
from examples_rclpy.publisher import MinimalPublisher
from examples_rclpy.subscriber import MinimalSubscriber
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor


def main_single(args=None):
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    # minimal_publisherノードの作成
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(minimal_publisher)
    executor.add_node(minimal_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    # Pythonクライアントライブラリの終了
    rclpy.shutdown()


def main_multi(args=None):
    # Pythonクライアントライブラリの初期化
    rclpy.init(args=args)
    # minimal_publisherノードの作成
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_publisher)
    executor.add_node(minimal_subscriber)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    minimal_publisher.destroy_node()
    minimal_subscriber.destroy_node()
    # Pythonクライアントライブラリの終了
    rclpy.shutdown()
