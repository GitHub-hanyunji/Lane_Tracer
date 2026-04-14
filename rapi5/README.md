# Lane Following
 
라인과 라인 사이의 영역(lane)을 따라 자율주행하는 패키지.
카메라 영상에서 좌·우 라인을 동시에 검출하고, 두 라인의 무게중심 중점을 기준으로
position error를 계산하여 좌·우 바퀴 속도를 제어한다.
 
영상 수신 → ROI 설정 → 좌/우 라인 검출 → 무게중심 중점 계산 → error 계산 → 속도 명령 발행
 
#### ▶ rapi5
https://github.com/GitHub-hanyunji/Lane_Tracer/tree/main/rapi5/lane_rapi5 <br>
launch.py 사용하여 camera_ros2, dxl_nano 패키지 구동

#### ▶ wsl
아래 READMD.md 파일에 상세설명 <br>
https://github.com/GitHub-hanyunji/Lane_Tracer/tree/main/rapi5/lane_wsl
 
### \<결과영상\>
[![Lane Following Robot](https://img.youtube.com/vi/IFIoyt00FPw/0.jpg)](https://www.youtube.com/watch?v=IFIoyt00FPw)
