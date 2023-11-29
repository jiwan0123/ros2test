import sys  # sys.argv 사용을 위해
from example_interfaces.srv import AddTwoInts  # AddTwoInts 서비스를 사용하기 위해
import rclpy  # rclpy 라이브러리를 사용하기 위해
from rclpy.node import Node  # Node 라이브러리를 사용하기 위해


class Client(Node):  # Client 클래스 선언
    def __init__(self):  # Client 클래스의 생성자 선언
        super().__init__('client')  # Node 클래스의 생성자 호출
        self.cli = self.create_client(AddTwoInts, 'server')  # 서버에 연결
        while not self.cli.wait_for_service(timeout_sec=1.0):  # 서버가 실행될 때까지 대기
            self.get_logger().info('Server not available, waiting again...')  # 로그 출력
        self.req = AddTwoInts.Request()  # AddTwoInts.Request 객체 생성

    def send_request(self):  # send_request 메소드 선언
        self.req.a = int(sys.argv[1])  # 서비스에 전달할 a 값 설정
        self.req.b = int(sys.argv[2])  # 서비스에 전달할 b 값 설정
        self.future = self.cli.call_async(self.req)  # 서비스에 요청


def main(args=None):  # main 함수 선언
    rclpy.init(args=args)  # rclpy 초기화
    client = Client()  # Client 객체 생성

    # 서비스 호출
    client.send_request()  # Client 객체의 send_request 메소드 호출

    # 서비스 응답 대기
    while rclpy.ok():  # rclpy가 실행되는 동안 반복
        rclpy.spin_once(client)  # client 객체를 계속 실행
        if client.future.done():  # 서비스 요청이 완료되었다면
            try:
                response = client.future.result()  # 서비스의 응답을 response에 저장
            except Exception as e:  # 예외 발생 시
                client.get_logger().info(  # 로그 출력
                    'Service call failed %r' % (e,)  # 로그 출력
                )
            else:
                client.get_logger().info(  # 로그 출력
                    'Result of add_two_ints: %d + %d = %d' %
                    (client.req.a, client.req.b, response.sum)
                )
            break

    client.destroy_node()  # client 객체 종료
    rclpy.shutdown()  # rclpy 종료


if __name__ == '__main__':
    main()
