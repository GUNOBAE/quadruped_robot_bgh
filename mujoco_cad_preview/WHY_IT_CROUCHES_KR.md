# BGH Quadruped — 왜 걷지 않고 주저앉는가?

목표: 기존 학습 파일을 지우거나 다시 학습하지 않고 원인을 분리해 조사한다. 이 도구는 시뮬레이션만 검사하며 Raspberry Pi / PCA9685 / 실물 서보 제어는 하지 않는다.

## 1. 기록으로 확인된 현상

전진 목표 +0.075 m/s, 6초 목표 이동 45cm일 때 PPO 실제 이동은 약 6.28cm이다.

| 항목 | 모든 Action 0 (기준) | PPO |
| --- | --- | --- |
| 최종 몸체 높이 | 10.10cm | 8.54cm |
| 최대 기울기 | 1.55도 | 27.60도 |
| 평균 접지 발 | 3.98 | 3.20 |
| 정규화 Action 평균 절대값 | 0 | 0.91 |
| 전진 거리 | -0.52cm | +6.28cm |
| 넘어짐 판정 | false | false |

fell=false는 좋은 자세의 증거가 아니다. 종료 조건은 높이 6cm 미만 또는 기울기 65도 초과이다. 따라서 상당히 웅크리거나 흔들려도 계속 실험할 수 있다.

Action 평균 0.91은 -1에서 +1 사이 정책 출력이 크다는 뜻이지 실제 토크 91%를 사용했다는 뜻이 아니다.

## 2. 왜 PPO 학습 점수가 높아도 걷지 못하는가

보상 함수에는 이동속도, 회전속도, 몸체 높이, 수평 자세, 부드러운 명령, 구동 일률, 발 미끄러짐 등이 함께 포함되어 있다.

예를 들어 좌회전 명령에서 목표 직선 속도는 0이므로 정지 상태도 선형 이동 추종 보상 최대 +2.0을 받을 수 있다. 회전 목표 달성에 대한 보상 최대값은 +0.7이다. 이로 인해 제자리 유지가 유리해질 가능성이 있다.

하지만 보상 함수가 유일한 원인이라고 단정하지 않는다. 임시 관절 매핑 오류, 작은 가동범위, 서보 목표각 추종 실패 등부터 확인해야 한다.

## 3. Windows에서 진단 실행

기존에 작동하던 경로에서 PowerShell을 실행한다.

    cd C:\quadruped_robot_bgh-feature-mujoco-cad-preview\mujoco_cad_preview

    $base="https://raw.githubusercontent.com/GUNOBAE/quadruped_robot_bgh/feature/rl-locomotion-diagnostics/mujoco_cad_preview"

    @("probe_actuators.py","compare_to_zero.py","RUN_WALK_DIAG.cmd") | ForEach-Object {
        Invoke-WebRequest "$base/$_" -OutFile $_
    }

    .\RUN_WALK_DIAG.cmd

원래 학습된 output/rl_runs/all_sim_ppo.zip과 output/physics_rl_v0.xml이 있어야 한다. 이 스크립트는 원래 학습 파일이나 CAD 메쉬를 수정하지 않는다.

### TEST A — 12개 모터를 가상 환경에서 한 개씩 움직이기

원래 Physics V1을 복사해 몸체를 고정하고 중력/접촉을 끈 다음, 각 제어축에 +0.12rad와 -0.12rad를 개별적으로 적용한다.

결과 파일: output/rl_runs/actuator_probe.json

주요 항목:

- max_foot_effect_cm: 한 모터를 시험했을 때 발끝이 실제로 이동한 최대 거리.
- joint_change_deg: 목표 약 6.88도를 관절이 실제로 따라간 각도.
- force_saturation_fraction: 시뮬레이션의 임시 토크 한계에 닿은 비율.
- max_constraint_gap_m: 가상 연결봉 양쪽 제약점의 최대 간격.
- small_foot_response_warning: 1mm 미만의 작은 반응 표시.

시험용 actuator가 실제 서보 채널과 정확히 연결된 것은 아니다. Fusion 관절 구조와 별도로 비교해야 한다.

### TEST B — PPO와 아무것도 하지 않는 기준 로봇 비교

같은 출발 상태와 같은 방향 명령에서, 0 동작과 PPO를 각각 6초간 실험한다.

결과 파일: output/rl_runs/zero_vs_ppo_all_sim_ppo_all.json

추가된 분석 지표:

- action_saturation_fraction: 정책 출력 절대값이 0.9 이상인 비율.
- mean_servo_tracking_error_deg: 목표각과 실제 관절각 사이 평균 오차.
- force_limit_fraction: 임시 토크 출력 한계에 닿은 시간 비율.
- fraction_feet_15mm_above_floor: 각 발이 15mm 이상 들린 시뮬레이션 시간 비율.
- max_toe_clearance_cm: 전체 발 중 최고 높이.
- normalized_primary_axis_gain: 목표 이동속도 대비 오차 개선 비율.
- crouch_vs_zero_m: 0 명령 상태보다 몸체가 더 낮아진 거리.
- cautious_command_tracking_candidate: 높이·기울기·추종 개선을 동시에 고려한 신중한 후보 판정.

candidate는 시뮬레이션의 연구용 지표일 뿐 진짜 걷기 검증이나 실물 사용 허가가 아니다.

## 4. 다음 의사결정

모터를 구동하면 적절한 발끝이 움직이는가? 실제 관절이 목표각을 따라가는가? 토크가 자주 포화되는가? 지지 발을 충분히 들어 올릴 수 있는가? 이 네 질문이 우선이다.

이 단계에서 실패했다면 PPO 학습 예산이나 Reward 가중치를 늘리는 것보다 모델 구조와 구동 제약을 먼저 고치는 게 맞다.

그다음 Reward를 서기/전진/회전별로 재설계하고, 독립된 다수 Episode에서 정량적으로 평가하자.

## 5. 공유할 파일

output/rl_runs/actuator_probe.json
output/rl_runs/zero_vs_ppo_all_sim_ppo_all.json

학습에 사용했던 원래 Policy ZIP은 재업로드하지 않아도 된다. 기존 모델을 직접 수정하는 작업도 없다.
