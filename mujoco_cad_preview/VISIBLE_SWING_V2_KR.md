# BGH Quadruped — Visible Swing V2 (22mm / 20mm)

## 기존 테스트가 왜 개미똥만큼만 움직였는가?

사용자가 보낸 ik_swing_report.json 및 ik_swing_trajectory.csv에서 확인:

- 기존 목표: 최대 상승 6mm, 전방 이동 5mm
- 실제 결과: 상승 6.087mm, 전방 5.035mm (IK 추종 성공)
- 목표 위치 최대 오차 약 0.413mm, 연결봉 제약점 오차 약 0.0046mm
- 기존 세 가상 모터의 목표각 최대 약 0.060rad = 3.4도

결론: 기구학 계산 자체는 잘 됐지만, 사용자 시연에 너무 작은 목표를 준 것이 문제다.
이 테스트를 반복할 이유는 없다.

## V2에서 확실히 바꾼 점

- 발 들기 22mm (이전보다 약 3.7배)
- 전방 20mm (이전보다 4배)
- Swing: 앞쪽으로 가며 발을 들어 올림
- Return: 바닥에 접촉하지 않고 시작점으로 돌아오기
- 발 주변 카메라 확대 (거리 0.22m)
- MuJoCo 창을 닫을 때까지 반복 재생
- 첫 1주기만 JSON과 CSV로 정량 평가
- 가상 서보 목표각은 ±0.235rad보다 커지지 않도록 제한
- 기존 Physics V1, 원본 CAD STL, 기존 PPO 정책 ZIP과 로그는 보존

중력 OFF, 지면 접촉 OFF, 몸체 고정. 이 Return은 진짜 지면 지지 보행이 아니다.

## Windows PowerShell 실행 방법

기존 정상 동작하는 폴더에서:

    cd C:\quadruped_robot_bgh-feature-mujoco-cad-preview\mujoco_cad_preview

    $base = "https://raw.githubusercontent.com/GUNOBAE/quadruped_robot_bgh/feature/visible-swing-v2/mujoco_cad_preview"
    @("swing_ik_v2.py","RUN_VISIBLE_SWING_V2.cmd","probe_actuators.py") | ForEach-Object {
        Invoke-WebRequest "$base/$_" -OutFile $_
    }
    .\RUN_VISIBLE_SWING_V2.cmd

메뉴에서 1번을 눌러 FL 앞왼다리를 먼저 선택한다.
초록 구체는 목표 발 위치이고, 실제 CAD 다리가 이를 따라가는지 확인한다.
창을 X로 닫으면 첫 주기의 결과가 저장된다.

출력 파일:
- output/ik_swing_visible_v2_report.json
- output/ik_swing_visible_v2_trajectory.csv
- output/ik_swing_visible_v2_fixed.xml

## 간략 모델 GitHub Actions 실제 검사

4개 다리 모두 Numerical IK 테스트 통과:

| 다리 | 상승 목표22mm / 실제 | 전방 목표20mm / 실제 |
| --- | ---: | ---: |
| FL | 23.07mm | 20.25mm |
| FR | 23.14mm | 20.24mm |
| BL | 23.07mm | 20.25mm |
| BR | 23.14mm | 20.24mm |

최대 목표 위치 오차 약 2.66mm, 폐루프 연결점 최대 오차 약 0.025mm.
다만 이 검사는 간략 기구학 모델 기반이며 실제 사용자 CAD 시각화의 최종 검증이 아니다.

## 왜 큰 움직임이 더 어렵나?

발끝과 관절 목표값의 국소적 관계는 DeltaP ≈ J DeltaQ,
따라서 DeltaQ ≈ pinv(J) DeltaP로 계산한다.
J는 MuJoCo 안에서 각 가상 모터를 조금씩 움직여 발끝 변화를 측정한 3x3 수치 Jacobian이다.

6mm 움직임에서는 선형 근사가 쉬웠지만, 22mm 스윙에서는 기구학적 비선형성이 커질 수 있다.
따라서 목표와 실제 발끝의 위치 오차 및 연결봉 제약 오차를 모두 기록한다.

## 다음 의사결정

V2를 실제 CAD로 확인하면 다음에는 더 이상 발끝만 흔드는 시험을 반복하지 않는다.
몸체의 freejoint를 활성화하고, 중력과 접촉을 켠 다음 나머지 3개 발로
지지한 채 한 발을 드는 실험으로 넘어간다.
그 이후 Crawl Gait와 보상 함수 개선/PPO 학습을 검토한다.

실물 서보의 모터 채널, 영점, 회전 방향, 속도/토크, 실리콘 접촉 물성이
아직 검증되지 않았으므로 가상 정책을 실물에 전송하지 않는다.
