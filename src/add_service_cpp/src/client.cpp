#include "rclcpp/rclcpp.hpp" // ROS2 기본 헤더파일
#include "example_interfaces/srv/add_two_ints.hpp" // 서비스 헤더파일

#include <chrono> // 시간 관련 헤더파일
#include <cstdlib> // C 표준 라이브러리 헤더파일
#include <memory> // 메모리 관련 헤더파일

using namespace std::chrono_literals; // 시간 단위 사용

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv); // 노드 초기화 argc: 입력 인자 개수, argv: 입력 인자 배열

    if (argc !=3) { // 입력 인자가 3개가 아니면
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y"); // 사용법 출력
        return 1; // 종료
    }

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("client"); // 노드 생성
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client = 
        node->create_client<example_interfaces::srv::AddTwoInts>("service"); // 매개변수로 받은 서비스 이름으로 클라이언트 생성

    auto req = std::make_shared<example_interfaces::srv::AddTwoInts::Request>(); // 서비스 요청 메시지 생성
    req->a = atoll(argv[1]); // 입력 인자를 서비스 요청 메시지에 저장
    req->b = atoll(argv[2]); // 입력 인자를 서비스 요청 메시지에 저장

    while (!client->wait_for_service(1s)) { // 서비스가 사용 가능할 때까지 대기
        if (!rclcpp::ok()) { // ROS2가 종료되면
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting."); // 에러 메시지 출력
            return 0; // 종료
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again..."); // 서비스가 사용 가능하지 않으면 메시지 출력
    }

    auto result = client->async_send_request(req); // 서비스 요청 메시지를 서비스 서버에 전송
    
    if (rclcpp::spin_until_future_complete(node, result) == 
        rclcpp::FutureReturnCode::SUCCESS) // 서비스 요청에 대한 응답이 오면
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum); // 서비스 응답 메시지 출력
    }
    else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints"); // 서비스 요청에 대한 응답이 오지 않으면 에러 메시지 출력
    }

    rclcpp::shutdown(); // 노드 종료
    return 0; // 종료
}