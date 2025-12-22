# Lane_Tracer
### 작성자: 2301510 한윤지
jeston_pub 패키지로 jetson에서 camrea frame publish 하고 wsl_sub패키지에서 subscrive 하여 좌/우 라인 추적하여 중심값의 error 계산 후 좌/우 바퀴 속도 계산하여 
publish하고 dxl_nano에서 실제로 바퀴 구동하는 프로그램

* 각 패키지에 대한 자세한 설명은 각 패키지 폴더 아래 README.md 파일로 자세히 작성하였습니다.
### pub 설명
- jetson 코드는 나중에 첨부하도록 하겠습니다.

### sub 설명
- camera frame subscrive하여 error로 좌/우 바퀴 속도 계산해 jetson dxl_nano에 publish
  
README: https://github.com/GitHub-hanyunji/Lane_Tracer/blob/main/README.md

### 결과영상 youtube
