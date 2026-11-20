# 참가자 수정 영역
패키지 이름은 hanyang_autonomy, launch 파일은 launch/participant.launch.py로 고정합니다.
인지·판단·제어, 설정, 모델, 여러 노드는 이 패키지 안에서 자유롭게 구성합니다.
실행 후 추가 입력 없이 작동해야 합니다.

```bash
ros2 launch hanyang_autonomy participant.launch.py
```

전용 명령의 stamp는 최신 센서 메시지의 stamp를 복사합니다.
ROS의 Unix 시각을 넣으면 Unity의 시계와 다르므로 거부됩니다.
원본 센서 출력은 고정되며 내부 축소·ROI·프레임 스킵은 자유입니다.
