# BGH Quadruped — MuJoCo Physics V1 (UNVALIDATED)

**현재 단계: CAD 형상 + 36개 관절 기구학 미리보기 성공 → 실제 중력/접촉 적용 첫 실험.**

> 주의: 이 버전은 물리 구성/낙하/관절 제어의 *진단용 시뮬레이터*입니다.
> 보행 학습, 실제 서기 성공, Sim2Real 또는 하드웨어 제어를 구현한 것이 아닙니다.
> 원본 CAD와 기존 정상 동작하는 MuJoCo 미리보기는 수정하지 않습니다.

## 1. 가장 쉬운 설치 (이미 CAD가 표시되던 Windows PC)

기존 정상 실행되는 폴더:
C:\quadruped_robot_bgh-feature-mujoco-cad-preview\mujoco_cad_preview

1. GitHub 브랜치 feature/mujoco-physics-v1에서 아래 3개 파일을 받습니다.
   - build_physics_v1.py
   - run_physics_v1.py
   - RUN_PHYSICS_V1.cmd
2. 세 파일을 기존 mujoco_cad_preview 폴더에 복사합니다.
3. RUN_PHYSICS_V1.cmd를 더블클릭합니다.
4. 명령 창의 heuristic_verdict를 확인합니다.
5. MuJoCo GUI에서 중력·발 접촉에 반응하는지 확인합니다.
6. output/physics_v1_report.json을 대화에 업로드합니다.

GitHub에서 파일 3개를 PowerShell로 받는 방법:

    cd C:\quadruped_robot_bgh-feature-mujoco-cad-preview\mujoco_cad_preview

    $base = "https://raw.githubusercontent.com/GUNOBAE/quadruped_robot_bgh/feature/mujoco-physics-v1/mujoco_cad_preview"

    "build_physics_v1.py", "run_physics_v1.py", "RUN_PHYSICS_V1.cmd" | ForEach-Object {
        Invoke-WebRequest "$base/$_" -OutFile $_
    }

    .\RUN_PHYSICS_V1.cmd

기존 output/cad_preview.xml, cad_meshes/*.stl, model_manifest.json,
cad_fit_report.json, audit_excerpt_reference.json 및 Python 가상환경을 재사용합니다.

## 2. 생성되는 파일

- output/physics_v1.xml: 자유 몸체 + 중력·접촉·센서가 있는 MJCF
- output/physics_v1_config.json: 임시 질량·관성·발 접촉 위치·서보 토크
- output/physics_v1_report.json: 물리 시뮬레이션 결과와 높이·기울기

기존 output/cad_preview.xml과 STL은 수정하지 않습니다.

## 3. 이번에 추가되는 물리 요소

| 항목 | 구현 |
|---|---|
| 몸체 이동 | freejoint 1개, 3D 이동 및 자세 |
| 중력 | 0, 0, -9.81m/s² |
| 충돌 | 평면 바닥 + 발끝 구체 4개 + 몸체 박스 1개 |
| 질량·관성 | CAD 추정값을 기준으로 한 **미검증 근사** |
| 서보 | 기존 12개 시험용 관절에 PD 제어, 임시 토크 제한 2.5Nm |
| 센서 | torso gyro / accelerometer / quaternion |
| 물리 계산 | MuJoCo mj_step 실행, 결과 기록 |
| 하드웨어 제어 | 없음 |

CAD 추정 전체 질량은 약 7.16kg이며, 실물 질량과 일치한다고 보장하지 않습니다.
서보 토크도 임시값이며 실제 SPT5435 정격을 반영한 값이 아닙니다.
연결봉은 기존 8개 폐루프 / 16개 equality connect로 연결됩니다.

몸체는 현재 CAD 관절 자세에서 시작하며, 발과 바닥을 약 7mm 띄웁니다.
처음 실행 시 넘어지거나 주저앉아도 정상적인 실험 결과입니다.
아직 서도록 학습시키지 않았기 때문입니다.

## 4. 결과 읽기

- NOT_STANDING_YET: 바닥 접촉·몸체 높이·기울기 중 하나 이상이 부족함.
- PRELIMINARY_STAND_CANDIDATE: 간단한 휴리스틱 조건 통과.
  **실제로 안정적인 Standing이 입증된 것은 절대 아닙니다.**
- maximum_simultaneous_foot_contacts: 시뮬레이션 중 동시에 접촉한 발 최대 개수
- last_second_min_height_m: 마지막 1초 동안 몸체 기준점 최저 높이
- last_second_max_tilt_deg: 마지막 1초 최대 몸체 기울기

오류가 발생하면 콘솔 로그 전체와 output/physics_v1_config.json 및
output/physics_v1_report.json을 보내주세요.

## 5. 실측 질량 반영 (선택)

본인 실물의 질량을 측정했다면 다음처럼 다시 생성할 수 있습니다.

    .\.venv\Scripts\python.exe build_physics_v1.py --total-mass-kg 4.6

4.6은 예시일 뿐입니다. 실제 측정값으로 바꿔주세요.

## 6. PPO 강화학습 전에 해야 할 것

1. 실제 MuJoCo 엔진에서 중력·발 접촉 계산 성공 확인.
2. 발바닥 실제 접촉 위치, 링크 간섭 및 관절 한계 확인.
3. **실제 12개 서보 구동 관절의 매핑·방향·중립각 검증.**
4. 실측 무게, 관성, 토크, 속도와 지연 반영.
5. Standing/정적 균형 실험 통과 및 넘어짐 기준 정의.
6. PPO 강화학습 환경, Domain Randomization 구성.
7. Pi 5 + IMU + 실제 서보 기반 안전 실험.

**현재 Physics V1은 1단계의 시작**이며 학습 완료 모델이 아닙니다.

## 7. GitHub 브랜치

- 기존 CAD 미리보기 코드: feature/mujoco-cad-preview / Draft PR #1
- 이 물리 검증 코드: feature/mujoco-physics-v1 / 별도 Draft PR
