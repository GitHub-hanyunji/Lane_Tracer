#ifndef LANE_DETECT_NODE_HPP
#define LANE_DETECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>                     // Node 상속
#include <sensor_msgs/msg/compressed_image.hpp>  // 콜백 파라미터 타입
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>                    

//LaneDetectNode 클래스
class LaneDetectNode : public rclcpp::Node
{
public:
    LaneDetectNode();  // 생성자

private:
    // 토픽 구독 Subscriber
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;
    int width, height;
    cv::Point p_center;   // line 추적용 중심점
    int l_idx=-1,r_idx=-1;
    cv::Point left_pt, right_pt;


    geometry_msgs::msg::Vector3 vel_msg;
    bool mode =false;
    double k = 0.23;

    void publish_frame();
    // callback
    void line_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    int getch(void);
    bool kbhit(void);
    void Set(cv::Mat& frame);   //roi 설정
    void Findline(cv::Mat& frame, cv
        ::Point& p_center, cv::Mat& stats, cv::Mat& centroids); // 라인추적함수
    void Draw(cv::Mat& frame,cv::Mat stats,cv::Mat centroids,int labels,int index,int r_idx,cv::Point p_center); // 시각화함수
};



// 함수

#endif // LINE_DETECT_NODE_HPP
