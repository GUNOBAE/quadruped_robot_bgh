# 다방향 Crawl + 병렬 보행 설정 최적화 V1

## 이 실행은 무엇인가?

PPO 신경망을 다시 처음부터 학습시키는 코드가 아니다. 기존 폐루프 IK·하중 보상 방식에 **평면 이동 방향과 몸체 목표 yaw**를 추가하고, 보폭/타이밍/몸체 이동 비용/경로 곡선을 병렬 비교하는 직접 최적화다. 결과는 neural policy ZIP이 아니라 방향별 제어 파라미터와 검증 기록이다.

기본 명령: forward, backward, left, right, forward_left, forward_right, backward_left, backward_right, turn_left, turn_right, stop.

대각선 벡터는 정규화해 두 축을 동시에 넣었다고 속도가 sqrt(2)배 늘어나지 않는다. 회전에서는 발 배치와 몸체 목표 방향을 함께 변경한다. 명령 전환 시험은 한 시뮬레이션의 상태를 유지한 채 걸음 주기 경계에서 수행한다. 순간적인 조이스틱 반응이나 보행 중 어느 시점에서든 즉시 전환하는 제어는 아직 아니다.

## 실행 전

- 이전 crawl_night / PPO / MuJoCo 뷰어는 종료한다. 다른 최적화가 실행 중이면 중복 실행을 거부한다.
- 기존 폴더의 .venv, linkage_core.py, output/physics_v1.xml, output/cad_meshes를 사용한다.
- 패키지를 재설치하거나 GPU 드라이버를 변경하지 않는다.
- 원본 CAD/물리 XML/과거 PPO/기존 16mm 설정 결과를 수정하지 않는다.

## 시작 (Windows PowerShell)

이 브랜치의 omni_trial.py, omni_parallel.py, OMNI_PARALLEL_KR.md를 기존 mujoco_cad_preview 폴더에 복사한 다음:

```powershell
.\.venv\Scripts\python.exe omni_parallel.py --start --hours 12 --workers 0 --trials 600
```

`--workers 0`은 CPU 논리 프로세서 수와 현재 사용 가능한 RAM을 읽어 보수적으로 자동 선택한다. 기본 자동 상한 4개, 자원이 부족하면 1개일 수 있다. 각 worker의 BLAS 스레드는 1개로 제한한다. 이것은 독립 시뮬레이션의 동시 실행이며 한 물리 시간축을 여러 조각으로 분할하는 것이 아니다. 4 workers가 정확히 4배 빠르다고 보장하지 않는다.

현재 구현은 CPU의 MuJoCo + SciPy IK를 사용한다. GPU PPO나 GPU-native 물리 엔진으로 자동 전환되지 않는다.

`--hours`는 전체 실행 예산의 상한이다(0.05~72시간). `--trials`는 후보 선별 수의 상한이다(1~1000). 모든 후보/검증이 끝나면 시간이 남아도 종료한다. 매번 새 결과 폴더를 만들며 기존 결과에 덮어쓰지 않는다.

### 콘솔 멈춤 방지

`--start`는 Windows에서 콘솔과 분리한 master를 실행한다. 입력은 DEVNULL, stdout/stderr는 master.log로 연결한다. worker도 창 없이 실행하고 각자의 console.log에 기록한다. 따라서 PowerShell 선택 모드가 계산 출력 통로를 막는 경로를 제거했다.

`STARTED PID ... WORKERS ...` 출력 후 `--status`로 실행 여부를 확인하면 **시작한 PowerShell 창은 닫아도 된다**. 이는 Windows 로그아웃/절전/종료 후에도 실행된다는 뜻이 아니다. 전원과 통풍을 확보하고 PC는 켜둔다. master는 실행 중 자동 절전 억제를 요청하고 종료 시 해제한다.

## 진행 상황 / 정지 / 결과

```powershell
.\.venv\Scripts\python.exe omni_parallel.py --status
.\.venv\Scripts\python.exe omni_parallel.py --stop
```

status.json은 약 3초 간격으로 갱신된다. active에는 각 worker의 PID, 작업명, 경과시간, 로그 경로가 나온다. --stop은 STOP 파일을 만들어 master에게 중지를 요청한다. 완료된 기록은 보존되며 진행 중인 한 작업의 미완성 기록은 통과로 인정하지 않는다. 재부팅 후 자동 재개 기능은 없다.

결과 폴더:

```powershell
$run = Get-Content .\output\omni_parallel\LATEST.txt
Invoke-Item $run
```

- SUMMARY.md: 최종 상태
- selection.json: 방향별 선택 설정과 검증 여부
- ranking.csv: 후보별 속도/기계적 일/몸체 흔들림 비교
- status.json / master.log / events.jsonl: 실행·동시 프로세스·종료 기록
- 각 작업의 report.json, trajectory.npz, telemetry.npz
- source.xml / code/: 실제 사용한 입력 복사본과 코드 스냅샷
- results_bundle.zip: 분석용 JSON/코드/소스 XML 묶음. 원본 CAD STL과 전체 궤적은 제외하므로 PC에 보관한다.

## 검증과 후보 선택

1. 10개 동작의 보수적 기준 제어기를 각각 3주기 검사한다. 실패한 방향은 장시간 최적화로 넘기지 않는다.
2. 통과한 방향의 1주기 기준과 후보를 같은 조건에서 비교한다. 명령 포화, 발 들기, 지지력, 착지 오차를 유지하면서 속도·몸체 이동량·가속도 변화를 기록한다.
3. 후보 상위 최대 2개와 해당 방향의 기준 설정을 각각 6주기 검사한다.
4. 통과한 설정을 마찰 0.9배/1.1배에서 각각 3주기 평가한다.
5. 기준 대비 개선 판정은 동일한 방향/주기/마찰 조건끼리 한다. 단기 성공만으로 자동 채택하지 않는다.
6. 확정된 동작들을 이어 전진/옆걸음/대각선/후진/회전/정지 전환도 별도로 검사한다.

선별 개선 조건은 기계적 일/거리(회전은 각도당 일)가 기준의 1.05배 이하이고, 안정성 조건과 함께 다음 중 하나다.
- 속도 최소 10% 개선, 몸체 측방 이동 진폭 악화 5% 이내, 가속도 RMS 악화 10% 이내.
- 속도는 기준의 90% 이상, 몸체 측방 이동 진폭 최소 15% 감소, 가속도 RMS 악화 5% 이내.

최종 선택은 검증 조건과 종합 점수를 모두 사용한다. 단기 개선 후보가 장기/마찰 검증에서 실패하거나 기준보다 종합 점수가 낮으면 기준 설정이 유지될 수 있다. 아무 방향도 확정되지 않으면 실패를 그대로 보고한다.

이 몸체 이동 진폭은 roll/pitch 기울기와 다르다. 이동 방향의 수직 방향으로 몸체 위치가 얼마나 오갔는지 측정한다. 회전 시험에서는 평면 몸체 위치 변동을 별도 근사 지표로 사용한다. 서로 다른 방향을 이 지표 하나로 직접 순위화하지 않는다.

기계적 일은 50Hz에서 측정한 sum(abs(torque * angular_velocity))의 적분이며 배터리 전력이나 전기 효율이 아니다. 정지 중 서보 유지 전류와 전자장치 손실은 포함하지 않는다.

## 화면 보기

검증을 통과해 selection.json에 선택된 기록만 재생한다.

```powershell
.\.venv\Scripts\python.exe omni_parallel.py --view left
.\.venv\Scripts\python.exe omni_parallel.py --view forward_left
.\.venv\Scripts\python.exe omni_parallel.py --view turn_left
.\.venv\Scripts\python.exe omni_parallel.py --view sequence
```

`--view sequence`는 명령 전환 검사까지 통과했을 때만 가능하다. 어느 방향이 검증을 통과하지 못했다면 그 방향을 성공한 것처럼 보여주지 않는다. 영상은 물리시험 기록의 **1배속 재생**이며 새로운 실시간 시뮬레이션이나 학습된 정책 추론은 아니다.

## 이번 개발에서 확인한 범위

사용자가 올린 results_bundle(1).zip의 source.xml을 그대로 사용했다. SHA256: c80ebdb4a37d720a9d2bd40990875417e1b7ee8d7d5abcb992950d9874141ecb.

MuJoCo 3.17.0 / Linux Python 3.13.5에서 전후좌우, 네 대각선, 좌우회전의 3주기 시험과 정지 시험이 모두 통과했다. 전진/후진은 보폭16mm, 나머지 평면 이동은6mm, 회전은 주기당1도로 작게 시작했다. 이 값은 속도 성능을 주장하기 위한 설정이 아니라 방향 제어 구현을 검증하기 위한 초기값이다.

한 시뮬레이션에서 초기화 없이 forward -> left -> forward_left -> stop -> backward -> right -> backward_right -> turn_left -> turn_right -> stop 순서도 통과했다.

별도 두 worker 전체 파이프라인 시험에서는 forward/left/turn_left 기준 설정의 6주기, 마찰 +/-10% 검증과 명령 전환 및 결과 압축 저장을 확인했다. 그 소량 탐색에서는 새 개선 후보가 아닌 기준 설정이 선택됐다. 따라서 이 구현만으로 속도와 흔들림이 이미 개선됐다고 주장하지 않는다.

## 남은 한계

- 여전히 느린 정적 crawl이고, 몸체 지지 이동을 완전히 없애거나 동적 trot을 구현한 것이 아니다.
- 각도/토크 제한, 질량/관성, 접촉체, kp/kv는 원본 값을 유지한다. 실측 물성으로 검증된 값이 아니다.
- 계획 앵커와 실제 착지 오차는 기록/실패 기준으로 관리하지만, 완전한 접촉 상태 추정기나 미끄러짐 관측기를 새로 구현한 것은 아니다.
- 실제 서보의 출력축/전기 채널/영점/회전 방향은 하드웨어 대조가 필요하다.
- 정확한 시뮬레이터 관절·몸체 상태를 사용하므로 IMU만 있는 실물로 그대로 이전할 수 없다.
- 기본 모델은 발/몸체 접촉 위주이며 모든 링크의 자기충돌을 검증하지 않는다.
- 병렬 탐색을 오래 돌린다고 모든 동작의 성능 개선이 보장되지 않는다.

참고: https://mujoco.readthedocs.io/en/stable/programming/simulation.html
참고: https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html
