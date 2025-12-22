#include "lane/lane.hpp"

int main(int argc, char * argv[])
{
    // ROS2 초기화
    rclcpp::init(argc, argv);
    // Node 실행 (spin)
    // 무한대기
    // 메시지 들어오면 콜백함수 호출
    rclcpp::spin(std::make_shared<LaneDetectNode>());
    rclcpp::shutdown();  // ROS2 종료
}
