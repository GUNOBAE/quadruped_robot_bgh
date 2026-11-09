# BGH Quadruped — 방향 명령 강화학습 PPO V0

**목표:** Fusion CAD 외형을 유지한 MuJoCo에서 전진, 후진, 왼쪽 옆걸음, 오른쪽 옆걸음, 좌회전, 우회전을 학습할 수 있는 실험 환경을 만든다.

> **시뮬레이션 전용.** PPO가 종료되거나 모델 ZIP이 생겼다고 보행이 성공한 것은 아니다. 현재 서보 매핑, 링크 물성, 실리콘의 정확한 탄성값은 확인되지 않았다. 학습된 정책을 Raspberry Pi 5 또는 실제 서보에 사용하지 않는다.

## 1. 시작 전 준비

현재 PC에서 실행 성공한 파일들이 필요하다.

- output/physics_v1.xml
- output/physics_v1_config.json
- output/cad_preview.xml
- output/cad_meshes/ (실제 Fusion STL 37개)
- .venv/Scripts/python.exe

이 브랜치는 이전 Physics V1 모델을 수정하지 않는다. 새로운 output/physics_rl_v0.xml을 생성한다.

### 가장 쉬운 설치 (Windows PowerShell)

기존 폴더를 유지한 채 아래 명령을 PowerShell에서 실행한다.

    cd C:\quadruped_robot_bgh-feature-mujoco-cad-preview\mujoco_cad_preview

    $base = "https://raw.githubusercontent.com/GUNOBAE/quadruped_robot_bgh/feature/mujoco-directional-rl-v0/mujoco_cad_preview"
    $names = @("build_rl_model.py","rl_env.py","check_rl.py","train_rl.py","evaluate_rl.py","requirements_rl.txt","RUN_RL_CHECK.cmd","RUN_RL_TRAIN.cmd","RUN_RL_DEMO.cmd")
    foreach ($name in $names) {
        Invoke-WebRequest "$base/$name" -OutFile $name
    }
    .\RUN_RL_CHECK.cmd

## 2. 실행 파일 세 개

**RUN_RL_CHECK.cmd**: 필요한 Stable-Baselines3, PyTorch, Gymnasium을 설치하고
실리콘 근사 모델을 만들고, 여섯 명령과 12개 구동 관절이 정상 연결되는지,
작은 PPO 학습을 실행할 수 있는지 확인한다.

**RUN_RL_TRAIN.cmd**: 다음 세 단계 학습을 순서대로 시도한다.
- A: 제자리 서기 — 20,000 결정
- B: 전진 / 후진 — 60,000 결정
- C: 전진 / 후진 / 좌 / 우 옆걸음 / 좌회전 / 우회전 — 180,000 결정

학습에는 CPU 연산이 사용되며 완료 시간과 품질을 보장하지 않는다.
학습 파일이 있으면 해당 단계는 건너뛰도록 구현되어 있다.

**RUN_RL_DEMO.cmd**: 방향별 수치 평가 또는 1개 방향 MuJoCo 화면을 연다.
학습 후 먼저 방향별 평가 결과를 읽는다.

학습 시작:

    .\RUN_RL_TRAIN.cmd

학습된 방향 화면:

    .\RUN_RL_DEMO.cmd

## 3. 좌표와 속도 명령

Fusion 모델 기준 X+ = 왼쪽, Y- = 전방, Z+ = 위쪽.
명령은 (전방 m/s, 왼쪽 m/s, 좌회전 rad/s) 3차원 벡터.

| 입력 | 명령 |
| --- | --- |
| forward | (0.075, 0, 0) |
| backward | (-0.075, 0, 0) |
| left | (0, 0.065, 0) |
| right | (0, -0.065, 0) |
| turn_left | (0, 0, 0.45) |
| turn_right | (0, 0, -0.45) |

속도는 초기 시험용 목표값이며 실제 속도나 성공을 뜻하지 않는다.
좌/우 옆걸음과 좌/우 회전은 서로 다르다.

## 4. 학습은 어떻게 작동하는가?

- AI는 20ms 간격으로 12개 관절의 목표 각도 명령을 선택한다.
- MuJoCo는 1ms 단위로 몸체 이동, 중력, 발 접촉을 계산한다.
- 목표 방향 속도에 가까워지면 보상을 받는다.
- 넘어지거나 몸체가 기울고 높이가 낮아지면 벌점을 받는다.
- 불필요한 관절 떨림, 일률, 발 미끄러짐도 벌점 대상이다.

관찰 모드:
- sim (기본): 실제 관절각/속도, 몸체 속도, IMU 유사 신호, 접촉 여부,
  이전 관절 명령, 원하는 방향 등 54개 값. **PWM 서보로는 이 정보를 모두 못 얻는다.**
- imu (별도 실험): 중력 방향/자이로, 이전 두 관절 명령, 이동 목표와 보행 위상
  35개 값만 관찰한다. 그래도 실물에 적용해도 된다는 뜻은 아니다.

## 5. 실리콘 발 근사

4개 발끝 접촉 구체에 마찰과 유연한 충돌 솔버를 적용했다.

- 초기 시험 슬라이딩 마찰계수 1.2
- 초기 접촉 시간상수 0.025초
- 감쇠비 1.0
- 학습 중 마찰 0.8~1.6 랜덤화
- 학습 중 접촉 시간상수 0.018~0.040초 랜덤화

**실제 실리콘 재료 측정값이 아니다.**
충돌 형태는 구체이며 패드가 눌려 납작해지거나 접촉면적이 변하는
실제 실리콘의 비선형 변형은 모델링하지 않았다.

## 6. 학습 결과 검증

생성 예정 파일:

- output/rl_v0_preflight.json : 환경 및 제자리 서기 기초 검사
- output/rl_runs/stand_sim_ppo.zip : 첫 단계 정책
- output/rl_runs/foreaft_sim_ppo.zip : 전진/후진 정책
- output/rl_runs/all_sim_ppo.zip : 여섯 방향 정책
- output/rl_runs/direction_evaluation.json : 방향별 이동 거리, 속도 오차, 전복 여부

**성공 판정은 반드시 평가 JSON과 MuJoCo 화면을 보고 한다.**
학습된 정책이 앞으로 이동하지 않거나 넘어지면 물리 파라미터,
실제 서보 구동 관절과 초기 자세부터 수정해야 한다.

## 7. 주의할 구조적 제약

현재 Physics V1의 12개 actuator는 시험용 관절이다.
실제 서보와 관절이 정확히 매핑됐다는 검증은 없다.
PCA9685만으로는 실제 관절 위치와 속도 데이터를 읽지 못한다.

실물 Sim2Real 전에 해결해야 할 것:
1. 다리 12개 실제 구동 축과 서보 채널, 회전 방향, 영점 확인.
2. 실제 총질량, 관성, 출력 토크, 속도 제한/지연 측정.
3. 실리콘 발의 실측 마찰과 하중-압축 특성 확인.
4. 실물에서 얻을 수 있는 관측값만 사용하도록 정책 재구성.
5. 보행 중 넘어짐 방지, 비상 정지 및 충분한 안전 여유 확보.

이 브랜치에서는 실제 로봇 I/O를 구현하지 않는다.

추가 문제 분석에 필요한 파일:
- output/physics_v1_report.json
- output/rl_v0_preflight.json
- output/rl_runs/direction_evaluation.json

## 8. 실기 결과 해석 — PPO 종료, 보행 0/6 (2026-11-10)

사용자 PC에서 약 261,120 환경 결정까지 PPO를 실행했고
전체 방향 평가 결과는 다음과 같았습니다 (6초 측정):

| 명령 | 목표 | 실제 이동/회전 | 판정 |
| --- | --- | --- | --- |
| forward | 45cm | 6.28cm | 실패 |
| backward | -45cm | -2.36cm | 실패 |
| left | 39cm | 0.43cm | 실패 |
| right | -39cm | -2.90cm | 실패 |
| turn_left | +2.70rad | -0.075rad | 실패 |
| turn_right | -2.70rad | -0.011rad | 실패 |

서기 정책을 stop 방향으로 평가하면 낙상 없이 거의 제자리에 있는
후보 수준이지만, **여섯 방향 모두 정상적인 보행 속도 추종을 못 합니다.**
평균 에피소드 길이가 400에 가까운 것은 8초 동안 버틴다는 의미일 뿐
걷기를 배웠다는 증거가 아닙니다.

### 8.1 흔들리다가 MuJoCo 창이 종료되는 현상

이전 evaluate_rl.py는 8초 에피소드가 끝나면 viewer context를 끝내면서
창을 자동으로 닫도록 작성되어 있었습니다. Python 에러가 아니었습니다.
현재 수정본은 `--view` 실행 중 에피소드가 끝나면 **같은 초기 조건으로
자동 재시작하여 사용자 스스로 창을 닫을 때까지 반복 재생**합니다.
숫자 리포트에는 첫 에피소드만 포함되어 비교 가능성을 유지합니다.

### 8.2 왜 보상이 올라가도 걷지 않는가?

기존 reward의 일부 항목은 제자리 자세를 유지하는 행동만으로도
높은 값을 줄 수 있습니다. 예를 들어 turn_left 명령의 이동 목표속도는
0이므로 정지 상태에서도 이동속도 추종 항목에서 **+2.0**을 받습니다.
회전속도 항목 가중치는 **0.7**이라 실제 회전보다 서서 버티는 행동이
점수 측면에서 유리할 수 있습니다. 이것만이 유일한 원인이라고 단정할
수는 없으며, 실제 12개 구동 관절과 가동폭/토크 제한도 검증해야 합니다.

### 8.3 학습 정책 vs 아무 명령도 하지 않는 기준 로봇 비교

새 compare_to_zero.py는 **같은 초기 상태, 같은 지면, 같은 명령**에서
AI 행동과 action=0을 비교합니다. PPO 정책이나 원본 CAD를 수정하지 않습니다.

기존 Windows 작업 폴더에서 최신 GitHub 스크립트 3개:

    cd C:\quadruped_robot_bgh-feature-mujoco-cad-preview\mujoco_cad_preview
    $base = 'https://raw.githubusercontent.com/GUNOBAE/quadruped_robot_bgh/feature/mujoco-directional-rl-v0/mujoco_cad_preview'
    @('evaluate_rl.py','rl_env.py','compare_to_zero.py') | ForEach-Object {
        Invoke-WebRequest "$base/$_" -OutFile $_
    }

기준 모델과 전체방향 PPO의 차이를 분석:

    .\.venv\Scripts\python.exe compare_to_zero.py --policy output\rl_runs\all_sim_ppo.zip --seconds 6

결과는 output/rl_runs/zero_vs_ppo_all_sim_ppo_all.json에 저장됩니다.
목표 방향 속도오차, 관절 명령 변동, 바닥 접촉, 낙상과 함께
**보상 항목별 합계와 단계별 평균값**을 기록합니다.
이 비교에서 PPO가 아무것도 하지 않는 정책보다 낫지 않다면
무작정 학습 횟수를 늘리기 전에 환경/관절/보상 설계를 재검토합니다.

전진 8초 에피소드를 계속 반복해서 화면에서 보기:

    .\.venv\Scripts\python.exe evaluate_rl.py --policy output\rl_runs\all_sim_ppo.zip --direction forward --seconds 8 --view

이제 창은 자동으로 닫히지 않으므로 **MuJoCo X 버튼**으로 직접 종료하세요.
평가 JSON은 각각 direction_evaluation_all_sim_ppo_forward.json 같은 별도 이름으로
보존되며, 기존 direction_evaluation.json은 마지막 평가로 갱신됩니다.

### 8.4 아직 하지 말아야 할 것

보상 계수나 서보 힘을 임의로 크게 변경하고 즉시 재학습하지 마세요.
먼저 실제 구동 관절 매핑/관절 출력축/회전 방향/발 접촉 위치/서보 토크를
확인하고, 물리 조건을 고정한 대조군 실험으로 문제가 어디에 있는지
판별하는 것이 우선입니다.
