#include "lane/lane.hpp"
#include <chrono>                 
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#define STDIN_FILENO 0

using namespace cv;

int LaneDetectNode::getch(void)
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

bool LaneDetectNode::kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

void LaneDetectNode::Set(Mat& frame)
{
    frame = frame(Rect(Point(0, frame.rows * 3 / 4), Point(frame.cols, frame.rows)));
    cvtColor(frame, frame, COLOR_BGR2GRAY);
    frame += Scalar(90) - mean(frame);
    threshold(frame, frame, 145, 255, THRESH_BINARY);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(frame, frame, kernel);   // 작은 노이즈 제거
    dilate(frame, frame, kernel);  // 라인 복원
}

void LaneDetectNode::Findline(Mat& frame, Point& p_center, Mat& stats, Mat& centroids)
{
    Mat labels;
    int cnt = connectedComponentsWithStats(frame, labels, stats, centroids);

    int left_idx = -1, right_idx = -1;
    double left_best = frame.cols, right_best = frame.cols;

    for (int i = 1; i < cnt; i++)
    {
        int area = stats.at<int>(i, 4);
        if (area > 500 || area <4500)
        {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));

            double d_left  = norm(Point(x, y) - left_pt);
            double d_right = norm(Point(x, y) - right_pt);

            if (d_left < left_best && d_left <= 100) {
                left_best = d_left;
                left_idx  = i;
            }
            if (d_right < right_best && d_right <= 100) {
                right_best = d_right;
                right_idx  = i;
            }
        }
    }

    bool left_ok  = (left_idx  != -1 && left_best  <= 100);
    bool right_ok = (right_idx != -1 && right_best <= 100);

    if (left_ok) {
        left_pt = Point(
            cvRound(centroids.at<double>(left_idx,  0)),
            cvRound(centroids.at<double>(left_idx,  1))
        );
    }
    if (right_ok) {
        right_pt = Point(
            cvRound(centroids.at<double>(right_idx, 0)),
            cvRound(centroids.at<double>(right_idx, 1))
        );
    }

    // ★ 핵심 수정: 재탐색 루프 제거, 1차 탐색 결과 바로 사용
    l_idx = left_ok  ? left_idx  : -1;
    r_idx = right_ok ? right_idx : -1;

    p_center = (left_pt + right_pt) / 2;

    if (!left_ok && !right_ok)
        circle(frame, p_center, 5, Scalar(0, 255, 0), -1);
}

void LaneDetectNode::Draw(Mat& frame, Mat stats, Mat centroids, int labels, int l_idx, int r_idx, Point p_center)
{
    cvtColor(frame, frame, COLOR_GRAY2BGR);

    bool l_valid = (l_idx >= 1 && l_idx < labels);
    bool r_valid = (r_idx >= 1 && r_idx < labels);

    for (int i = 1; i < labels; i++)
    {
        int area = stats.at<int>(i, 4);
        if (area < 500)  // ★ 수정: Findline과 기준 통일 (200→100)
            continue;

        Scalar color;
        if      (l_valid && i == l_idx) color = Scalar(0, 0, 255);
        else if (r_valid && i == r_idx) color = Scalar(0, 0, 255);
        else                            color = Scalar(255, 0, 0);

        rectangle(frame,
            Rect(stats.at<int>(i,0), stats.at<int>(i,1),
                 stats.at<int>(i,2), stats.at<int>(i,3)),
            color, 2);
        circle(frame,
            Point(centroids.at<double>(i,0), centroids.at<double>(i,1)),
            3, color, -1);
    }

    circle(frame, p_center, 5, Scalar(0, 255, 0), -1);

    if (!l_valid || !r_valid)
    {
        rectangle(frame, Rect(left_pt.x  - 2, left_pt.y  - 2, 4, 4), Scalar(0, 0, 255), 2);
        rectangle(frame, Rect(right_pt.x - 2, right_pt.y - 2, 4, 4), Scalar(0, 0, 255), 2);
    }
}

LaneDetectNode::LaneDetectNode() : Node("linedetect_wsl")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    auto fn  = std::bind(&LaneDetectNode::line_callback, this, std::placeholders::_1);
    sub  = this->create_subscription<sensor_msgs::msg::CompressedImage>("/image/compressed_13", qos, fn);
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos);

    width  = 640;
    height = 480;

    p_center = Point(width / 2,     (height / 4) / 2);
    // left_pt  = Point(width / 4,     (height / 4) / 2);
    // right_pt = Point(width * 3 / 4, (height / 4) / 2);
    left_pt  = Point(width / 6,     (height / 4) / 2);
    right_pt = Point(width * 5 / 6, (height / 4) / 2);

    vel_msg.x = 0.0;
    vel_msg.y = 0.0;
    vel_msg.z = 0.0;

    RCLCPP_INFO(this->get_logger(), "Lane Detect Node Started");
}

void LaneDetectNode::line_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    auto start = std::chrono::steady_clock::now();

    cv::Mat frame = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    if (frame.empty()) return;

    cv::Mat roi = frame.clone();
    Set(roi);

    cv::Mat labels, stats, centroids;
    Findline(roi, p_center, stats, centroids);
    Draw(roi, stats, centroids, stats.rows, l_idx, r_idx, p_center);

    int error = (roi.cols / 2) - p_center.x;

    // ★ 수정: 타임스탬프를 imshow 전에 찍어서 처리시간만 측정
    auto end = std::chrono::steady_clock::now();
    float t  = std::chrono::duration<float, std::milli>(end - start).count();

    float leftvel  =  100 - k * error;
    float rightvel = -(100 + k * error);

    char ch;
    if (kbhit())
    {
        ch = getch();
        if      (ch == 'q') mode = false;
        else if (ch == 's') mode = true;
    }

    if (mode) {
        vel_msg.x = leftvel;
        vel_msg.y = rightvel;
        vel_msg.z = 0.0;
    } else {
        vel_msg.x = 0.0;
        vel_msg.y = 0.0;
        vel_msg.z = 0.0;
    }

    pub_->publish(vel_msg);
    RCLCPP_INFO(this->get_logger(), "err:%d lvel:%f rvel:%f time:%f",
                error, vel_msg.x, vel_msg.y, t);

    cv::imshow("frame", frame);
    cv::imshow("Track", roi);
    cv::waitKey(1);
}