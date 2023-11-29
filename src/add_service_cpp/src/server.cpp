#include "rclcpp/rclcpp.hpp" // ROS2 기본 헤더파일
#include "example_interfaces/srv/add_two_ints.hpp" // 서비스 헤더파일

#include <memory> // 메모리 관련 헤더파일

void callback(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> req, // 매개변수는 서비스 요청 메시지 포인터와 서비스 응답 메시지 포인터 
std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> res) // 서비스 요청에 대한 응답을 처리하는 콜백 함수
{
    res->sum = req->a + req->b; // 서비스 요청 메시지의 a와 b를 더해서 서비스 응답 메시지의 sum에 저장
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld", // 서비스 요청 메시지의 a와 b를 출력
    req->a, req->b);
    RCLCPP_INFO(rclcpp::get_logger("rcldcpp"), "sending back response: [%ld]", (long int) // 서비스 응답 메시지의 sum을 출력
    res->sum);
}

int main(int argc, char **argv) // 노드 메인 함수 argc: 입력 인자 개수, argv: 입력 인자 배열
{
    rclcpp::init(argc, argv); // 노드 초기화

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("server"); // 노드 생성

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv = 
        node->create_service<example_interfaces::srv::AddTwoInts>("service", &callback); // 매개변수로 받은 서비스 이름으로 서비스 생성

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints."); // 서비스가 사용 가능하면 메시지 출력

    rclcpp::spin(node); // 노드 실행
    rclcpp::shutdown(); // 노드 종료
}