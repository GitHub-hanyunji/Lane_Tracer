## Wsl
### 작성자: 2301510 한윤지

Jetson에서 Publish한 메시지를 Mat으로 변환하여 영상 프레임에서 좌/우 라인을 찾고 그 사이 중심점의 오류(error)를 계산하는 노드이다.
계산한 error로 좌/우 속도 계산하여 jetson의 dxl_nano로 publish 한다.
2️ROI 설정 → 이진화 → 객체 분석 → 좌/우 각각 가장 가까운 라인 추적 → 시각화 후 에러 값 계산 → 에러값으로 좌/우 속도 계산 → 속도 publish

#### 1. Set 함수
입력 영상의 하단 1/4만 잘라서 ROI로 만들고, Gray 변환 + 밝기 보정 + 이진화까지 수행하여 라인 검출하기 좋은 형태로 전처리하는 함수이다.
```sh
// 관심영역 설정 함수
void LaneDetectNode::Set(Mat& frame){
    frame = frame(Rect(Point(0, frame.rows * 3 / 4), Point(frame.cols, frame.rows)));  // ROI 하단 1/4
    cvtColor(frame, frame, COLOR_BGR2GRAY);  // Gray스케일
    frame += Scalar(100) - mean(frame);  // 밝기보정
    // 이진화 (범위 조정=>150이 제일 적당하다고 판단)
    threshold(frame, frame, 160, 255, THRESH_BINARY);
}
```

#### 2. Findline 함수
ROI 이진영상에서 connectedComponentsWithStats로 차선(blob)을 검출하고, 좌/우 라인 각각 이전 중심점과 가장 가까운 객체를 선택해 차로 중심을 갱신한 뒤 추적된 blob의 인덱스를 반환하는 차선 추적 함수이다.
```sh
//라인 추적 함수
// 좌/우 라인 찾아 그 사이 중심점 찾는 함을 차로중심으로 계산하는 함수
void LaneDetectNode::Findline(Mat& frame, Point& p_center, Mat& stats, Mat& centroids) {
    Mat labels;
    // connectedComponentsWithStats로 라인영역 찾기 -> 배경 포함한 blob 개수
    //stats: 각 객체의 bounding box + area
    //centroids: 각 blob의 무게중심
    int cnt = connectedComponentsWithStats(frame, labels, stats, centroids);

    // 우리의 중심점과 가장 가까운 객체 찾기
    int left_idx = -1, right_idx = -1;
    double left_best = frame.cols, right_best = frame.cols;
    //int mid_x = frame.cols / 2;


    // 0번은 배경이니까 제외하고 반복문
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4); // 객체 면적
        if (area > 100) 
        { // 객체 중심점 계산 
            int x = cvRound(centroids.at<double>(i, 0)); 
            int y = cvRound(centroids.at<double>(i, 1)); 
            // 우리의 중심점과 좌/우객체 중심의 거리 구하기 
            double d_left = norm(Point(x,y) - left_pt); 
            double d_right = norm(Point(x, y) - right_pt); 
            //150px 이내에 있고 지금까지 본 객체들 중 가장 가까우면 그것을 후보로 저장 
            if (d_left < left_best && d_left <= 150) { 
                left_best = d_left; left_idx = i; 
            } 
            if (d_right < right_best && d_right <= 150) { 
                right_best = d_right; right_idx = i; 
            } 
        } 
    }
    // 좌/우 차선 검출 성공 여부 판단
    bool left_ok  = (left_idx  != -1 && left_best  <= 150);
    bool right_ok = (right_idx != -1 && right_best <= 150);
    // 왼쪽 -> 차선 위치 갱신
    if (left_ok) {
        left_pt=Point(
            cvRound(centroids.at<double>(left_idx, 0)),
            cvRound(centroids.at<double>(left_idx, 1))
        );
    }
    else l_idx = -1;
    // 오른쪽 -> 차선 위치 갱신
    if (right_ok) {
        right_pt=Point(
            cvRound(centroids.at<double>(right_idx, 0)),
            cvRound(centroids.at<double>(right_idx, 1))
        );
    }
    else r_idx = -1;

    // 좌/우 차선의 중간
    // 로봇이 따라가야 할 중심
    p_center = (left_pt + right_pt) / 2;

    // 차선 둘 다 없을때 마지막 중심점 유지
    if(!left_ok&&!right_ok){
        circle(frame, p_center, 5, Scalar(0, 255, 0), -1); // 차로 중심
    }

    // 가장 가까운 객체 재탐색
    double l_best = frame.cols;
    double r_best = frame.cols;
    for(int i = 1; i < stats.rows; i++)
    {
        int cx = centroids.at<double>(i, 0);
        int cy = centroids.at<double>(i, 1);
        double d = norm(Point(cx, cy) - left_pt);  // 우리 중심점과 객체 중심점 거리 구하기
    
        if (d < l_best)
        {
            l_best = d;
            l_idx = i;
        }
    }
    for(int i = 1; i < stats.rows; i++)
    {
        int cx = centroids.at<double>(i, 0);
        int cy = centroids.at<double>(i, 1);
        double d = norm(Point(cx, cy) - right_pt);  // 우리 중심점과 객체 중심점 거리 구하기
    
        if (d < r_best)
        {
            r_best = d;
            r_idx = i;
        }
    }
    // 어느 정도 거리 이내일 때만 index 인정
    if (l_best > 30)   // 30px 이상 멀면 tracking 실패 처리
        l_idx = -1;
    if(r_best>30)
        r_idx=-1;
}
```

#### 3. Draw 함수
검출된 라인 후보들에 대해 바운딩 박스와 중심점을 색깔(빨강=추적 중, 파랑=기타)으로 표시하고,
유효한 라인을 못 찾으면 기준 중심점 위치를 빨간 박스로 표시하는 시각화 함수이다
```sh
// 라인 시각화 함수
void LaneDetectNode::Draw(Mat& frame, Mat stats, Mat centroids,int labels, int l_idx, int r_idx, Point p_center)
{
    // 이진 이미지를 컬러 이미지로 변환
    cvtColor(frame, frame, COLOR_GRAY2BGR);

    // index 유효성 체크
    // labels->stats.rows
    // vaild=true-> 추적, vaild=false->추적x
    bool l_valid = (l_idx >= 1 && l_idx < labels);
    bool r_valid = (r_idx >= 1 && r_idx < labels);

    // 배경 건너 뜀
    for (int i = 1; i < labels; i++)
    {
        // 면적
        int area = stats.at<int>(i, 4);
        // 너무 작거나 큰 노이즈 skip
        if (area < 200)
            continue;
        
        Scalar l_color,r_color,color;
        // 빨강: 내가 주행하는 라인, 파랑=내가 주행하지 않는 라인
        if(l_valid && i == l_idx)
            color=Scalar(0,0,255);
        else if(r_valid && i == r_idx)
            color=Scalar(0,0,255);
        else
            color=Scalar(255,0,0);
         


        // 바운딩 박스
        rectangle(frame,Rect(stats.at<int>(i,0), stats.at<int>(i,1),stats.at<int>(i,2), stats.at<int>(i,3)),color, 2);
        // 중심점    
        circle(frame,Point(centroids.at<double>(i,0), centroids.at<double>(i,1)),3, color, -1);
        circle(frame, p_center, 5, Scalar(0, 255, 0), -1); // 차로 중심
    }

    // 객체 못찾았을때
    if (!l_valid||!r_valid)
    {
        // 라인 다시 찾았을 때 중심점 기준으로 다시 움직이기 위해서 빨간색으로 표시 => 중심점 주변 빨간 네모 표시
        rectangle(frame,Rect(left_pt.x - 2, left_pt.y - 2, 4, 4),Scalar(0,0,255), 2);
        rectangle(frame,Rect(right_pt.x - 2, right_pt.y - 2, 4, 4),Scalar(0,0,255), 2);
        circle(frame, p_center, 5, Scalar(0, 255, 0), -1); // 차로 중심
    }
}
```

#### 4. LaneDetectNode 생성자
LaneDetectNode 클래스의 생성자.
압축 영상 토픽 /image/compressed_13을 subscrive하고 초기 ROI 중심점과 화면 크기를 초기화하는 노드이다.
```sh
// LaneDetectNode 클래스
// 생성자
// rclcpp::Node를 상속한 클래스
LaneDetectNode::LaneDetectNode() : Node("linedetect_wsl")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));   // qos
    auto fn=std::bind(&LaneDetectNode::line_callback, this, std::placeholders::_1);
    sub = this->create_subscription<sensor_msgs::msg::CompressedImage>("/image/compressed_13",qos,fn);
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("dxl/line",qos);

    width = 640;
    height = 360;
    // ROI 중앙
    p_center = Point(width / 2, (height/4)/2);
    //p_center = Point(width / 2, (height / 4) / 2);
    left_pt=Point(width/4,(height/4)/2);
    right_pt=Point(width*3/4,(height/4)/2);
    //real_center=cv::Point();

    vel_msg.x = 0.0;
    vel_msg.y = 0.0;
    vel_msg.z = 0.0; // 필요 없으면 0

    RCLCPP_INFO(this->get_logger(), "Line Detect Node Started");
}

```

#### 5. Callback 함수
Set, Findline, Draw 함수를 사용하여 압축된 영상 메시지를 OpenCV Mat으로 복원해 ROI 처리·라인 추적·시각화를 수행하고, 중심선 기준 오차와 프레임 처리 시간을 계산해 출력한 후 error값으로 바퀴 좌/우 속도 계산하여 publish하는 콜백 함수이다.
매 프레임마다 원본 화면과 ROI 결과를 imshow로 띄우며 실시간으로 추적 상황을 확인할 수 있다.
```sh
// callback 함수
void LaneDetectNode::line_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 타임 스탬프
    // 프레임 처리 속도 측정하기 위한 시작 타임스탬프
    auto start = std::chrono::steady_clock::now();  

    // 압축된 이미지 메시지를 Mat으로 변환
    cv::Mat frame = cv::imdecode(msg->data, cv::IMREAD_COLOR);
    if(frame.empty()) return;

    cv::Mat roi = frame.clone();
    Set(roi);  // roi 설정

    cv::Mat labels, stats, centroids;
    // 라인 추적
    Findline(roi, p_center, stats, centroids);  



    // 시각화
    Draw(roi, stats, centroids, stats.rows, l_idx,r_idx, p_center);

    int center_x = roi.cols / 2;

    // 에러 계산
    // 관심영역 중심-차로중심
    int error = center_x - p_center.x;
    
    //프레임 처리 시간을 초 단위로 변환
    auto end = std::chrono::steady_clock::now();
    float t = std::chrono::duration<float,std::milli>(end - start).count();

    // 좌/우 속도 계산
    float leftvel = 100 - k* error;
    float rightvel = -(100 + k* error);
    
    char ch;
    if(kbhit()) // 없으면 제어 멈춤
    {
        ch = getch();
        // 주행 중지
        if(ch == 'q'){
            mode=false;
        }
        // 주행 시작
        else if(ch == 's'){
            mode=true;
        }
    }
    // 주행 중이면 속도값  적용
    if(mode){
        vel_msg.x = leftvel;
        vel_msg.y = rightvel;
        vel_msg.z = 0.0; // 필요 없으면 0
    }
    // 주행중이 아니면 속도값 0 적용
    else if(!mode){
        vel_msg.x = 0.0;
        vel_msg.y = 0.0;
        vel_msg.z = 0.0; // 필요 없으면 0
    }
    pub_->publish(vel_msg);  // 속도 값 publish
    RCLCPP_INFO(this->get_logger(), "err:%d lvel:%f rvel:%f time:%f", error,vel_msg.x,vel_msg.y, t);
    cv::imshow("frame", frame);  // 원본
    cv::imshow("Track", roi);    // ROI
    cv::waitKey(1);

}

```
