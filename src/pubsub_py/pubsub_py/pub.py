# rclpy 모듈 및 std_msgs모듈 import
import rclpy  # rclpy는 ros2와 상호작용하는 파이썬 api를 제공하는 모듈
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String


class Pub(Node):  # publisher 노드 클래스 Node클래스 상속
    def __init__(self):  # pub 클래스 생성자
        super().__init__('pub')  # super 함수를 통해 Node 클래스의 생성자에 노드 이름을 매게변수로 작성

        qos_profile = QoSProfile(depth=10)
        # String타입의 메시지를 'topic'이름의 토픽으로 publish함 ,
        # create_publisher 함수를 사용해 publisher 생성, 매개변수로는 메시지 타입, 토픽이름, QoS임
        self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
        timer_period = 0.5  # second 타이머 주기
        # create_timer함수를 통해 타이머 생성, 매개변수는 타이머 주기, 콜백함수
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.count
        self.publisher_.publish(msg)  # 위의 msg를 publish함수를 사용하여 publish함
        self.get_logger().info('Publishing: "%s"' % msg.data)  # 콘솔 로그 표시
        self.count += 1


def main(args=None):
    rclpy.init(args=args)  # 주어진 args에 대한 ROS통신 초기화
    node = Pub()  # pub 클래스
    try:
        rclpy.spin(node)  # 프로그램이 종료될 때 까지 큐에 요청된 콜백함수 처리
    except KeyboardInterrupt:
        node.get_logger().into('keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()  # 노드 종료 시 destroy_node 함수를 통해 publish 노드 소멸
        rclpy.shutdown()  # shutdown 함수를 통해 노드 종료


if __name__ == '__main__':
    main()