#include <memory> // 스마트 포인터 관련 라이브러리

#include "rclcpp/rclcpp.hpp" // ROS 2 C++ 클라이언트 라이브러리
#include "std_msgs/msg/string.hpp" // ROS 2 표준 메시지 라이브러리
using std::placeholders::_1; // std::bind 사용을 위한 네임스페이스

// 구독자 노드 클래스
class Sub : public rclcpp::Node
{
public:
    Sub() : Node("sub") /** @brief Sub 클래스의 생성자 */
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); // 메시지 큐의 크기를 10개로 설정한 qos_profile 생성
        subscriber_ = this->create_subscription<std_msgs::msg::String>( // 토픽 이름은 topic, 메시지 큐의 크기는 10개로 설정한 구독자 생성
            "topic", qos_profile, std::bind(&Sub::listener_callback, this, _1)); // topic_callback 함수를 호출
    }

private:
    void listener_callback(const std_msgs::msg::String::SharedPtr msg) const // topic_callback 함수 정의
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str()); // 메시지를 출력
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; // 구독자 객체
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // ROS 2 라이브러리 초기화, 매개변수는 main 함수의 매개변수를 그대로 전달
    auto node = std::make_shared<Sub>(); // Sub 클래스의 객체를 생성
    rclcpp::spin(node); // 노드 실행
    rclcpp::shutdown(); // 노드 종료
    return 0;
}