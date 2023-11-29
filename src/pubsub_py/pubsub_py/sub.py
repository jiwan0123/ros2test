# rclpy 모듈 및 std_msgs모듈 import
import rclpy  # rclpy 모듈 및 std_msgs모듈 import
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String


class Sub(Node):  # subscriber 노드 클래스 Node클래스 상속
    def __init__(self):  # sub 클래스 생성자
        super().__init__('sub')  # super 함수를 통해 Node 클래스의 생성자에 노드 이름을 매게변수로 작성
        qos_profile = QoSProfile(depth=10)
        # String타입의 메시지를 'topic'이름의 토픽으로 subscribe함 ,
        # create_subscription 함수를 사용해 subscriber 생성, 매개변수로는 메시지 타입, 토픽이름,콜백함수, QoS임
        self.subscriber_ = self.create_subscription(
            String, 'topic', self.listener_callback, qos_profile)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)  # 주어진 args에 대한 ROS통신 초기화
    node = Sub()  # sub 클래스
    try:
        rclpy.spin(node)  # 프로그램이 종료될 때 까지 큐에 요청된 콜백함수 처리
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()  # 노드 종료 시 destroy_node 함수를 통해 subscriber 노드 소멸
        rclpy.shutdown()  # shutdown 함수를 통해 노드 종료

    if __name__ == '__main__':
        main()