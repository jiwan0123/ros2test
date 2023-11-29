import rclpy  # rclpy 라이브러리를 사용하기 위해 import
from rclpy.action import ActionClient  # ActionClient 라이브러리를 사용하기 위해 import
from rclpy.node import Node  # Node 라이브러리를 사용하기 위해 import

# tutorial_interfaces.action 라이브러리에서 Fibonacci 라이브러리를 사용하기 위해 import
from tutorial_interfaces.action import Fibonacci


class Client(Node):  # Client 클래스 선언
    def __init__(self):  # Client 클래스의 생성자 선언
        super().__init__('client')  # Node 클래스의 생성자 호출
        self._action_client = ActionClient(
            self, Fibonacci, 'action')  # ActionClient 객체 생성

    def send_goal(self, order):  # send_goal 메소드 선언
        goal_msg = Fibonacci.Goal()  # Fibonacci.Goal 객체 생성
        goal_msg.order = order  # order 값 설정

        self._action_client.wait_for_server()  # ActionClient 객체가 서버를 찾을 때까지 대기

        self._send_goal_future = self._action_client.send_goal_async(  # ActionClient 객체에 goal을 전송
            # feedback_callback 메소드를 콜백으로 설정
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(
            self.goal_response_callback)  # goal_response_callback 메소드를 콜백으로 설정

    def goal_response_callback(self, future):  # goal_response_callback 메소드 선언
        goal_handle = future.result()
        if not goal_handle.accepted:  # goal_handle이 accepted 상태가 아니라면
            self.get_logger().info('Goal rejected :(')  # 로그 출력
            return

        self.get_logger().info('Goal accepted :)')  # 로그 출력

        # goal_handle의 결과를 받아오기 위해 get_result_async 메소드 호출
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(
            self.get_result_callback)  # get_result_callback 메소드를 콜백으로 설정

    def get_result_callback(self, future):  # get_result_callback 메소드 선언
        result = future.result().result  # future의 결과를 result에 저장
        self.get_logger().info('Result: {0}'.format(result.sequence))  # 로그 출력
        rclpy.shutdown()  # rclpy 종료

    def feedback_callback(self, feedback_msg):  # feedback_callback 메소드 선언
        feedback = feedback_msg.feedback  # feedback_msg의 feedback을 feedback에 저장
        self.get_logger().info(  # 로그 출력
            'Received feedback: {0}'.format(feedback.partial_sequence))  # 로그 출력


def main(args=None):  # main 함수 선언
    rclpy.init(args=args)  # rclpy 초기화

    client = Client()  # Client 객체 생성
    client.send_goal(10)  # Client 객체의 send_goal 메소드 호출

    rclpy.spin(client)  # client 객체를 계속 실행


if __name__ == '__main__':  # main 함수가 main 모듈로 실행되었는지 확인
    main()  # main 함수 호출
