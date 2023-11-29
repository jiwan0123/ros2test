cmake_minimum_required(VERSION 3.8) //cmake 최소 버전
project(add_service_cpp) //프로젝트 이름

if(NOT CMAKE_CXX_STANDARD) //CMAKE_CXX_STANDARD가 없으면
  set(CMAKE_CXX_STANDARD 14) //CMAKE_CXX_STANDARD를 14로 설정
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") //CMAKE_COMPILER_IS_GNUCXX가 참이거나 CMAKE_CXX_COMPILER_ID가 Clang이면
  add_compile_options(-Wall -Wextra -Wpedantic) //컴파일 옵션 추가
endif()

# find dependencies
find_package(ament_cmake REQUIRED) //ament_cmake 패키지 찾기
find_package(rclcpp REQUIRED) //rclcpp 패키지 찾기
find_package(example_interfaces REQUIRED) //example_interfaces 패키지 찾기

# Build
add_executable(server src/server.cpp) //server.cpp를 빌드
ament_target_dependencies(server rclcpp example_interfaces) //server에 rclcpp와 example_interfaces 패키지 의존성 추가

add_executable(client src/client.cpp) //client.cpp를 빌드
ament_target_dependencies(client rclcpp example_interfaces) //client에 rclcpp와 example_interfaces 패키지 의존성 추가

# Install
install(TARGETS
  server 
  client
  DESTINATION lib/${PROJECT_NAME} //빌드한 파일을 lib/add_service_cpp에 설치
) //TARGETS에 있는 파일들을 DESTINATION에 있는 경로에 설치

if(BUILD_TESTING) //BUILD_TESTING가 참이면
  find_package(ament_lint_auto REQUIRED) //ament_lint_auto 패키지 찾기
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies() //테스트 의존성 추가
endif()

ament_package() //ament 패키지