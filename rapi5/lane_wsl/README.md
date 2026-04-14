# Lane Following

### 작성자: 2301510 한윤지

카메라 영상(`CompressedImage`)을 구독하여 좌·우 라인을 동시에 검출하고,
두 라인의 무게중심 중점을 기준으로 position error를 계산하여
좌·우 바퀴 속도를 `topic_dxlpub`으로 발행하는 Lane Following 노드이다.

영상 수신 → ROI 설정 → 이진화 → 좌/우 라인 검출 → 중점 계산 → error 계산 → 속도 발행

---

## 파일 구성

| 파일 | 설명 |
|---|---|
| `lane.hpp` | 클래스 선언, 멤버 변수 및 함수 정의 |
| `lane.cpp` | 각 함수 구현부 |
| `main.cpp` | ROS2 초기화 및 노드 실행 |

---

## 클래스 구조 (`lane.hpp`)

`rclcpp::Node`를 상속하는 `LaneDetectNode` 클래스이다.
`/image/compressed_13` 토픽을 구독하고 `topic_dxlpub` 토픽을 발행한다.

```cpp
#ifndef LANE_DETECT_NODE_HPP
#define LANE_DETECT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <opencv2/opencv.hpp>

class LaneDetectNode : public rclcpp::Node
{
public:
    LaneDetectNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_;

    int width, height;            // frame 크기 (640×480)
    cv::Point p_center;           // 좌·우 라인 무게중심의 중점
    int l_idx = -1, r_idx = -1;   // 좌/우 라인 컴포넌트 인덱스
    cv::Point left_pt, right_pt;  // 좌/우 라인 추적 기준점
    geometry_msgs::msg::Vector3 vel_msg;  // 발행할 속도 메시지
    bool mode = false;            // 주행 모드 플래그
    double k = 0.23;              // 비례 제어 이득

    void line_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    int getch(void);
    bool kbhit(void);
    void Set(cv::Mat& frame);
    void Findline(cv::Mat& frame, cv::Point& p_center,
                  cv::Mat& stats, cv::Mat& centroids);
    void Draw(cv::Mat& frame, cv::Mat stats, cv::Mat centroids,
              int labels, int l_idx, int r_idx, cv::Point p_center);
};

#endif
```

---

## 함수 설명

#### 1. 생성자 `LaneDetectNode()`
`/image/compressed_13` 토픽을 구독하고 `topic_dxlpub` 발행자를 초기화한다.
frame 크기(640×480)와 좌·우 라인 초기 탐색 기준점을 설정한다.

```cpp
LaneDetectNode::LaneDetectNode() : Node("linedetect_wsl")
{
    sub  = this->create_subscription<sensor_msgs::msg::CompressedImage>(
           "/image/compressed_13", qos, fn);
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos);

    width = 640; height = 480;
    p_center  = Point(width / 2,     (height / 4) / 2);
    left_pt   = Point(width / 6,     (height / 4) / 2);  // x축 1/6 지점
    right_pt  = Point(width * 5 / 6, (height / 4) / 2);  // x축 5/6 지점
    vel_msg.x = vel_msg.y = vel_msg.z = 0.0;
}
```

---

#### 2. `Set()` 함수
입력 영상의 하단 1/4만 잘라서 ROI로 만들고, 그레이스케일 변환 → 밝기 보정 →
이진화 → 침식·팽창으로 노이즈를 제거하는 전처리 함수이다.

```cpp
void LaneDetectNode::Set(Mat& frame)
{
    // 하단 1/4 ROI 추출
    frame = frame(Rect(Point(0, frame.rows * 3 / 4), Point(frame.cols, frame.rows)));

    cvtColor(frame, frame, COLOR_BGR2GRAY);          // 그레이스케일 변환
    frame += Scalar(90) - mean(frame);               // 밝기 보정 (평균 기반)
    threshold(frame, frame, 145, 255, THRESH_BINARY); // 이진화

    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(frame, frame, kernel);   // 작은 노이즈 제거
    dilate(frame, frame, kernel);  // 라인 복원
}
```

---

#### 3. `Findline()` 함수
ROI 이진 영상에서 연결된 컴포넌트를 분리한 뒤,
이전 프레임의 좌·우 기준점에서 가장 가까운 컴포넌트를 각각 좌·우 라인으로 선택한다.
두 라인의 무게중심 중점을 `p_center`로 갱신한다.

```cpp
// 탐색 반경: 100px 이내
// 면적 필터: 500px 이상 4500px 이하
// 좌/우 각각 가장 가까운 컴포넌트 1개씩 선택
// p_center = (left_pt + right_pt) / 2
```

| 항목 | 값 |
|---|---|
| 탐색 반경 | 100px |
| 면적 필터 | 500 ~ 4500px |
| 초기 좌 기준점 | x = width/6 |
| 초기 우 기준점 | x = width×5/6 |

---

#### 4. `Draw()` 함수
검출된 컴포넌트에 바운딩 박스와 무게중심 점을 그리고,
좌·우 라인으로 선택된 컴포넌트는 빨간색, 나머지는 파란색으로 표시한다.
중점 `p_center`는 초록색 원으로 표시한다.

```cpp
// 좌/우 라인 선택된 컴포넌트 → 빨간색 박스
// 나머지 컴포넌트          → 파란색 박스
// p_center                → 초록색 원
// 라인 미검출 시          → left_pt / right_pt 위치에 빨간 사각형
```

---

#### 5. `line_callback()` 함수
`/image/compressed_13` 토픽으로 수신된 CompressedImage를 디코딩하여
Lane Following 알고리즘을 수행하고 속도를 발행하는 핵심 제어 루프이다.

```cpp
void LaneDetectNode::line_callback(...)
{
    // 1. CompressedImage 디코딩 → OpenCV Mat
    // 2. Set(roi)      → ROI + 이진화
    // 3. Findline()    → 좌/우 라인 검출 + p_center 갱신
    // 4. Draw()        → 시각화
    // 5. error = (roi.cols / 2) - p_center.x
    // 6. leftvel  =   100 - k * error
    //    rightvel = -(100 + k * error)
    // 7. kbhit() 감지
    //      's' → mode=true  (주행 시작)
    //      'q' → mode=false (주행 정지)
    // 8. mode=true  → vel_msg = (leftvel, rightvel, 0)
    //    mode=false → vel_msg = (0, 0, 0)
    // 9. pub_->publish(vel_msg)
}
```

---

#### 6. `getch()` / `kbhit()` 함수
엔터 없이 키보드 입력을 즉시 감지하기 위한 터미널 제어 함수이다.

```cpp
int  LaneDetectNode::getch();   // 키 하나 즉시 읽기
bool LaneDetectNode::kbhit();   // 키 입력 대기 여부 확인 (non-blocking)
```

---

## main.cpp

ROS2를 초기화하고 `LaneDetectNode`를 실행한다.
`rclcpp::spin()`으로 무한 대기하며 `/image/compressed_13` 메시지가 들어올 때마다 콜백을 호출한다.

```cpp
#include "lane/lane.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                              // ROS2 초기화
    rclcpp::spin(std::make_shared<LaneDetectNode>());      // 노드 실행 (무한 대기)
    rclcpp::shutdown();                                    // ROS2 종료
}
```

---

## error 계산

```
error = (영상 중심 x) - (좌측 라인 무게중심 x + 우측 라인 무게중심 x) / 2
      = roi.cols / 2 - p_center.x
```

| error | 의미 | 동작 |
|---|---|---|
| 0 | 정중앙 | 직진 |
| 양수 (+) | 로봇이 왼쪽으로 치우침 | 우회전 |
| 음수 (-) | 로봇이 오른쪽으로 치우침 | 좌회전 |

---

## 키보드 제어

| 키 | 동작 |
|---|---|
| `s` | 주행 시작 |
| `q` | 주행 정지 |

---

## 토픽

| 방향 | 토픽명 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/image/compressed_13` | `sensor_msgs/CompressedImage` | 카메라 영상 |
| 발행 | `topic_dxlpub` | `geometry_msgs/Vector3` | 좌·우 바퀴 속도 (x=lvel, y=rvel) |
