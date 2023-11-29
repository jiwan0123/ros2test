import time  # time.sleep()을 사용하기 위해 time 모듈을 import

import rclpy  # rclpy 라이브러리를 사용하기 위해 import
from rclpy.action import ActionServer  # ActionServer 라이브러리를 사용하기 위해 import
from rclpy.node import Node  # Node 라이브러리를 사용하기 위해 import

# tutorial_interfaces.action 라이브러리에서 Fibonacci 라이브러리를 사용하기 위해 import
from tutorial_interfaces.action import Fibonacci


class Server(Node):  # Server 클래스 선언
    def __init__(self):  # Server 클래스의 생성자 선언
        super().__init__('server')  # Node 클래스의 생성자 호출
        self._action_server = ActionServer(  # ActionServer 객체 생성
            self,  # Node 객체
            Fibonacci,  # Fibonacci 객체
            'action',  # Action 이름
            self.callback  # callback 메소드를 콜백으로 설정
        )

    def callback(self, goal_handle):  # callback 메소드 선언
        self.get_logger().info('Executing goal...')  # 로그 출력

        feedback_msg = Fibonacci.Feedback()  # Fibonacci.Feedback 객체 생성
        feedback_msg.partial_sequence = [0, 1]  # partial_sequence 값 설정

        for i in range(1, goal_handle.request.order):  # goal_handle.request.order만큼 반복
            feedback_msg.partial_sequence.append(  # partial_sequence에 값 추가
                # 피보나치 수열 계산
                feedback_msg.partial_sequence[i] + \
                feedback_msg.partial_sequence[i-1]
            )
            self.get_logger().info('Feedback: {0}'.format(
                feedback_msg.partial_sequence))  # 로그 출력
            # feedback_msg를 goal_handle에 전송
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # 1초 대기

        goal_handle.succeed()  # goal_handle을 성공으로 설정

        result = Fibonacci.Result()  # Fibonacci.Result 객체 생성
        result.sequence = feedback_msg.partial_sequence  # result.sequence 값 설정

        return result


def main(args=None):  # main 함수 선언
    rclpy.init(args=args)  # rclpy 초기화

    server = Server()  # Server 객체 생성

    rclpy.spin(server)  # server 객체를 계속 실행


if __name__ == '__main__':  # main 함수가 main 모듈로 실행되었는지 확인
    main()  # main 함수 실행
