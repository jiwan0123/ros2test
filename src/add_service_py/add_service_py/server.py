from example_interfaces.srv import AddTwoInts  # AddTwoInts 서비스를 사용하기 위해

import rclpy  # rclpy 라이브러리를 사용하기 위해
from rclpy.node import Node  # Node 라이브러리를 사용하기 위해


class Server(Node):  # Server 클래스 선언

    def __init__(self):  # Server 클래스의 생성자 선언
        super().__init__('server')  # Node 클래스의 생성자 호출
        # func parameter(MassageType, TopicName, Callback function)
        self.srv = self.create_service(AddTwoInts, 'server', self.callback)

    def callback(self, req, res):  # callback 메소드 선언
        res.sum = req.a + req.b  # res.sum에 req.a와 req.b를 더한 값을 저장
        self.get_logger().info('Incoming request\n a: %d b: %d' % (req.a, req.b))  # 로그 출력

        return res  # res 반환


def main(args=None):  # main 함수 선언

    rclpy.init(args=args)  # rclpy 초기화

    server = Server()  # Server 객체 생성

    rclpy.spin(server)  # server 객체를 계속 실행

    rclpy.shutdown()  # rclpy 종료


if __name__ == '__main__':
    main()
