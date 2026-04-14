# launch.py (rapi5)

### 작성자: 2301510 한윤지

`camera_ros2`와 `dxl_nano` 두 개의 노드를 동시에 실행하는 런치 파일이다.
카메라 영상을 발행하고, WSL2의 lane following 노드에서 계산된 속도 명령을 구독하여 Dynamixel을 구동한다.

---

## 실행 노드 구성

| 패키지 | executable | 노드 이름 | 역할 |
|---|---|---|---|
| `camera_ros2` | `pub` | `campub` | 카메라 영상 토픽 발행 |
| `dxl_nano` | `sub` | `node_dxlsub` | 속도 명령 토픽 구독 → Dynamixel 구동 |

---

## 실행 방법

```sh
ros2 launch <패키지명> launch.py
```

---

## 코드

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros2',
            executable='pub',
            name='campub',
            output='screen',
        ),
        Node(
            package='dxl_nano',
            executable='sub',
            name='node_dxlsub',
            output='screen',
        )
    ])
```

---

## 노드 설명

#### 1. campub 노드 (`camera_ros2`)
카메라 영상을 캡처하여 `CompressedImage` 토픽으로 발행하는 노드이다.
WSL2의 lane following 노드가 이 토픽을 구독하여 라인 검출을 수행한다.

```python
Node(
    package='camera_ros2',
    executable='pub',
    name='campub',
    output='screen',
)
```

---

#### 2. node_dxlsub 노드 (`dxl_nano`)
속도 명령 토픽을 구독하여 Dynamixel 모터를 구동하는 노드이다.
WSL2의 lane following 노드가 발행한 `topic_dxlpub` 토픽을 구독하고,
수신된 `l_vel`, `r_vel` 값에 따라 좌·우 바퀴를 제어한다.

```python
Node(
    package='dxl_nano',
    executable='sub',
    name='node_dxlsub',
    output='screen',
)
```

---

## 토픽 흐름

```
[campub] → /image/compressed_13 → [lane (WSL2)] → topic_dxlpub → [node_dxlsub] → Dynamixel
```
