cmake_minimum_required(VERSION 3.8) // CMake 최소 버전 지정
project(tutorial_interfaces) // CMake 프로젝트 이름 지정

if(NOT CMAKE_CXX_STANDARD) // C++ 표준 지정
  set(CMAKE_CXX_STANDARD 14) // C++14 표준 사용
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") // 컴파일러가 GCC 또는 Clang인 경우
  add_compile_options(-Wall -Wextra -Wpedantic) // 컴파일 옵션 추가
endif()

# find dependencies
find_package(ament_cmake REQUIRED) // ament_cmake 패키지 찾기
find_package(builtin_interfaces REQUIRED) // builtin_interfaces 패키지 찾기
find_package(rosidl_default_generators REQUIRED) // rosidl_default_generators 패키지 찾기

rosidl_generate_interfaces(${PROJECT_NAME} // rosidl_generate_interfaces() 함수 호출
  "msg/Num.msg" // Num.msg 파일 추가
  "srv/AddThreeInts.srv" // AddThreeInts.srv 파일 추가
  "action/Fibonacci.action" // Fibonacci.action 파일 추가
  DEPENDENCIES builtin_interfaces // 의존성 추가
)

ament_export_dependencies(rosidl_default_runtime) // rosidl_default_runtime 패키지 의존성 추가
ament_package() // ament_package() 함수 호출

