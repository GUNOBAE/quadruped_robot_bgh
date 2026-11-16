# BGH — 학습 없는 링크식 저속 Crawl V1

## 이번에 달라진 것

이 코드는 새로운 PPO 실험이나 몸체 고정 발끝 애니메이션이 아니다. 기존 Physics V1의 자유 몸체, 중력, 발 접촉, 질량, 관성, 서보 제어 이득과 각도/토크 제한을 그대로 읽는다. 실시간 물리 상태는 12개 position actuator의 ctrl과 mj_step으로만 진행한다. 몸체 좌표를 직접 바꾸거나 외력을 가해 밀지 않는다.

기본 동작은 `몸체 지지 자세 이동 -> 한 발 하중 이관 -> 발 들기 -> 앞/뒤로 이동 -> 접촉 확인`이다. FL -> BR -> FR -> BL 순서로 한 발씩 옮긴다. 먼저 느린 전진/후진의 기준 동작을 확보하는 코드이며, 옆걸음/회전/실물 보행이 검증된 것은 아니다.

## 1. 기존 폴더에서 실행

새 Python 환경이나 패키지 업데이트는 필요 없다. 기존 MuJoCo, NumPy, SciPy를 사용한다. 기존 `output/physics_v1.xml`과 `output/cad_meshes`가 있어야 한다.

PowerShell에서:

```powershell
cd C:\quadruped_robot_bgh-feature-mujoco-cad-preview\mujoco_cad_preview
$ErrorActionPreference = 'Stop'
$base = 'https://raw.githubusercontent.com/GUNOBAE/quadruped_robot_bgh/feature/linkage-baseline-v1/mujoco_cad_preview'
@('linkage_core.py','baseline_crawl.py','BASELINE_CRAWL_KR.md') | ForEach-Object {
    Invoke-WebRequest -UseBasicParsing "$base/$_" -OutFile "$_.download"
    Move-Item "$_.download" $_ -Force
}
.\.venv\Scripts\python.exe -m py_compile linkage_core.py baseline_crawl.py
if ($LASTEXITCODE -ne 0) { throw 'Python syntax check failed.' }
.\.venv\Scripts\python.exe baseline_crawl.py --cycles 3 --view
```

CMD 배치 파일을 사용하지 않아 이전 괄호 파싱 오류를 피한다. 전진과 후진을 각각 검사한 다음, 통과한 물리시험의 기록을 CAD 외형으로 재생한다. 실패한 결과를 성공한 걸음처럼 재생하지 않는다.

화면은 **기록된 물리 계산의 재생**이다. 최초 검증에서는 중력/접촉과 서보 입력으로 mj_step을 수행했고, 뷰어에서는 그 결과 qpos 기록을 반복해서 보여준다. 화면에서 qpos를 읽어 표시하는 재생 코드와 실제 물리시험에서 qpos를 직접 수정하는 것은 다르다. 실제 시험에서는 root pose나 joint pose를 강제로 지정하지 않는다.

## 2. 실제 검증 결과와 범위

검증 자료는 사용자가 업로드한 `ik_swing_fixed.xml`이다. 이 파일의 원래 링크 구조·축·질량·관성·발/몸체 충돌체·서보 제한을 유지하고, 고정 시험에서 제거했던 base_free를 복원하고 중력/접촉을 켰다. 표시용 목표구체만 제거했다. 원본 CAD는 공개 저장소에 올리지 않았다. 이것은 누락된 DayLab source.xml을 그대로 재실행한 시험이라고 주장하지 않는다.

MuJoCo 3.17.0 / Linux / Python 3.13에서 두 방향 모두 3주기를 검증했다. MuJoCo 3.3.7에서도 같은 시험을 수행해 유사 결과를 확인했다. 사용자의 Windows GUI는 별도로 확인해야 한다.

| 항목 | 전진 | 후진 |
|---|---:|---:|
| 목표 순이동 | +36.0 mm | -36.0 mm |
| 측정 순이동 | +36.32 mm | -35.79 mm |
| 옆방향 드리프트 | 0.72 mm | 0.50 mm |
| 최대 몸체 기울기 | 1.69 deg | 1.82 deg |
| 발 최대 지면 간격 | 각 스텝 13.81~14.12 mm | 13.94~14.31 mm |
| 들기/세 발 지지/착지 검사 | 12/12 | 12/12 |
| 최대 폐루프 제약점 간격 | 0.592 mm | 0.589 mm |
| 평균 명령 제한 적용 비율 | 2.12% | 2.74% |
| 평균 토크 포화 비율 | 0.11% | 0% |
| 시뮬레이션 시간 | 66.4 s | 66.4 s |

여기서 순이동은 처음 준비 자세 이동을 제외하고 측정한다. 이 속도는 매우 느리며 이전 목표 40mm/s를 달성한 결과가 아니다. `SIMULATED_CRAWL_PASS`는 이 유한한 평지 시험의 기준을 만족했다는 뜻이다. 일반적인 안정성 인증이나 실물 동작 보장이 아니다.

유지한 값: 총질량 7.16404kg, position kp=20/kv=1, 명령 범위 +/-0.25rad, 가상 서보 토크 +/-2.5N m. 이 수치는 기존 모델의 임시값이며 실측 자료가 아니다. 명령 제한/토크 포화가 완전히 사라진 것도 아니다. 포화 비율은 모든 모터와 기록 시간에 대해 평균한 값이다.

## 3. 제어 원리

### A. 닫힌 기구 전체를 푸는 IK

이전에는 초기 자세에서 모터를 조금 움직여 만든 하나의 3x3 근사행렬을 계속 사용했다. 이번에는 MJCF에 있는 각 링크 위치와 관절축을 이용해 28개 tree hinge 좌표를 구하고, 16개 connect 제약으로 표현된 8개 폐루프를 동시에 만족시킨다. 발끝 위치 오차와 루프 접합점 오차를 매번 검사한다.

Zuko가 기하학적 다리각과 실제 연결봉 전달을 분리하는 설계 원리는 참고했다. 그러나 Zuko의 링크 길이나 servo 영점은 복사하지 않았다. 이번 계산의 기하학 데이터는 사용자의 MJCF다. 이 사실만으로 전기적 서보 채널/극성/영점이 맞다고 증명되는 것은 아니다.

### B. 세 발로 버티기 위한 지지 자세

다리 하나를 들기 전에 나머지 세 발이 받아야 할 힘을 계산한다. 몸체 위치 후보를 비교해 음수 지지력이나 과한 모터 부하가 생기는 자세를 피한다. 이 계산은 제한된 정적 자세 최적화이고, PPO 학습이나 물리 파라미터 변경이 아니다.

### C. 무게를 버틸 힘을 위치 명령에 반영

MuJoCo position actuator는 목표 위치를 강제로 실현하지 않는다. 대략 `tau = kp*(ctrl-q) - kv*qdot`로 힘을 만든다. 따라서 하중을 받으면 실제각이 목표각보다 처질 수 있다.

이번 제어기는 수동 관절의 폐루프 관계를 제거한 가상일 방정식으로 지지력을 계산하고, 필요한 토크를 위치 명령 보정으로 사용한다:

`ctrl = q_IK + tau_support/kp`

이후 소스에 있는 명령/토크 한계 안에서만 작동한다. 몸체 위치·자세 오차도 제한된 PI 보정으로 줄인다. 들어 올리는 발은 실제 몸체 자세를 이용해 세계좌표 목표를 따라가며, 지지 발은 바닥의 앵커 위치를 유지한다.

### D. 실제 접촉을 통과 기준으로 사용

발을 들었다고 인정하려면 지면 간격이 8mm를 넘고 해당 발의 수직 접촉력이 1N 미만인 시간이 최소 0.2초 있어야 한다. 그 동안 나머지 세 발은 각각 2N 이상의 지지를 보여야 한다. 착지 후 접촉력도 확인한다. 단순히 뒤로 밀린 거리를 보행 성공으로 세지 않는다.

## 4. 파일과 결과

실행마다 별도 폴더를 만든다:

`output/baseline_crawl/날짜_시간/`

- `summary.json`: 전체 결과, 소스 SHA256, 파일 보존 여부
- `forward/report.json`, `backward/report.json`: 방향별 통과/실패 이유와 지표
- 각 방향의 `telemetry.json`: 발별 접촉력/위치, 몸체, 목표각/실제각, 토크, 제약 오차
- 각 방향의 `trajectory.npz`: 실제 물리 계산에서 기록한 qpos
- `source.xml`: 재현을 위한 입력 복사본, 원본 변경 없음
- `baseline_results.zip`: 분석용 전체 묶음. 원본 STL은 포함하지 않음

결과 폴더 열기:

```powershell
$run = Get-Content .\output\baseline_crawl\LATEST.txt
Invoke-Item $run
```

다시 후진 기록 보기:

```powershell
.\.venv\Scripts\python.exe baseline_crawl.py --view-only --direction backward
```

## 5. 남아 있는 한계

- 실제 12개 서보 채널/출력축/극성/중립각은 아직 하드웨어 검증 전이다.
- 전체 질량·관성·서보 응답·실리콘 발 특성은 실제 측정값이 아니다.
- 제어는 정확한 시뮬레이터 몸체 위치·관절 상태를 사용한다. IMU만 있는 실물로 바로 옮길 수 없다.
- 발과 몸체만 충돌체를 가진 기존 모델이다. 모든 링크의 자기충돌/간섭을 검증한 것이 아니다.
- 측정 가능한 범위의 느린 전진/후진만 검증했다. 요철·외란·다른 지면·빠른 속도·옆걸음·회전·장기안정성은 별도 시험 대상이다.
- 이번 비교는 여러 제어 요소를 함께 바꿨다. 각 요소의 독립적인 기여도를 분리한 ablation 결과는 아니다.

실물로 전송하는 코드는 없고, 기존 학습된 PPO를 재사용하거나 수정하지 않는다.

## 근거와 참고

- 사용자가 제공한 ASSAMBLE_A 관련 joint audit, ik_swing_fixed.xml, DayLab 결과 ZIP
- Zuko linked-leg transmission separation: https://github.com/reubenstr/zuko/blob/main/quad_ws/src/quad_main/quad_main/src/kinematics.py
- MuJoCo mj_jacSite / mj_contactForce: https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html
- MuJoCo position actuator: https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-position
- Spot Mini Mini baseline gait + residual-learning methodology: https://github.com/OpenQuadruped/spot_mini_mini
