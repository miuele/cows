cmake_minimum_required(VERSION 3.8)
project(cogtwa_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/ODReadU16.srv"
    "srv/ODRead.srv"
    "srv/ODWrite.srv"
    "srv/SDOReadU16.srv"
    "srv/SDORead.srv"
    "srv/SDOWrite.srv"
    "srv/ConfigureODNotify.srv"
    "srv/NMTStateGetByNodeId.srv"
    "srv/TPDOUseUnused.srv"
    "srv/RPDOUseUnused.srv"
    "srv/HBConsumerSetByNodeId.srv"
    "msg/ODWriteNotify.msg"
    "msg/NMTState.msg"
    "msg/HBConsumerEvent.msg"
    "msg/ODSink.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
