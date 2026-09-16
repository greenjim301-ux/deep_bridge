cmake_minimum_required(VERSION 3.10)
project(deep_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
)

# 后台接收线程用 std::thread，显式链接 pthread
find_package(Threads REQUIRED)

# 指南 1.1.2：本体默认在 DTLS（UDP + 加密）端口上通信，见 UdpTransport
find_package(OpenSSL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(deep_bridge_node
        src/main.cpp
        src/CmdVelBridge.cpp
        src/UdpTransport.cpp
        include/deep_bridge/CmdVelBridge.h
        include/deep_bridge/UdpProtocol.h
        include/deep_bridge/UdpTransport.h
        include/deep_bridge/json.hpp
)

add_dependencies(deep_bridge_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(deep_bridge_node
        ${catkin_LIBRARIES}
        Threads::Threads
        OpenSSL::SSL
        OpenSSL::Crypto
)

install(TARGETS deep_bridge_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        OPTIONAL
)
