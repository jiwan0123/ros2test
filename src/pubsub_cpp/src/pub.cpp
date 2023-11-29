#include <chrono>  // 시간 관련 라이브러리
#include <functional>  // 함수 객체 관련 라이브러리
#include <memory>  // 스마트 포인터 관련 라이브러리
#include <string>  // 문자열 관련 라이브러리

#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ 클라이언트 라이브러리
#include "std_msgs/msg/string.hpp"  // ROS 2 표준 메시지 라이브러리

using namespace std::chrono_literals;  // 시간 리터럴 사용을 위한 네임스페이스

// @brief Pub 클래스는 퍼블리셔 노드를 생성하고 초기화하는 역할을 담당합니다.
class Pub : public rclcpp::Node{ //Node 클래스를 상속받아 Pub 클래스를 정의
    public:
        Pub() : Node("pub"), count(0) //Node 클래스의 생성자를 호출하고 노드 이름을 pub로 설정
        {
            auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)); //메시지 큐의 크기를 10개로 설정
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos_profile); //토픽 이름은 topic, 메시지 큐의 크기는 10개
            timer = this->create_wall_timer(500ms, std::bind(&Pub::timer_callback, this)); //500ms마다 timer_callback 함수를 호출
        }

    private:
        void timer_callback() //timer_callback 함수 정의
        {
            auto msg = std_msgs::msg::String(); //std_msgs::msg::String 타입의 메시지 객체 생성
            msg.data = "Hello, world! " + std::to_string(count++); //메시지 객체의 data 멤버에 문자열을 저장
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str()); //메시지를 출력
            publisher_->publish(msg); //메시지를 퍼블리시
        }
        rclcpp::TimerBase::SharedPtr timer; //타이머 객체
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; //퍼블리셔 객체
        size_t count;
        
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); //ROS 2 라이브러리 초기화, 매개변수는 main 함수의 매개변수를 그대로 전달
    auto node = std::make_shared<Pub>(); //Pub 클래스의 객체를 생성
    rclcpp::spin(node); //노드 실행
    rclcpp::shutdown(); //노드 종료
    return 0; 
}