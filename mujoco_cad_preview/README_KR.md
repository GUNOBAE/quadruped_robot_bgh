# BGH Quadruped — 메인 PC 처음 설치 / Fusion CAD → MuJoCo

> **처음 읽을 문서입니다.** 이전 PC의 quadruped_v01 폴더나 ChatGPT 다운로드 파일이 없어도,
> GitHub의 코드로 **관절 미리보기 데이터를 새로 생성**할 수 있도록 복구했습니다.
> 하지만 **사용자 소유 원본 CAD(STL) 파일은 GitHub에 공개하지 않았습니다.**
> CAD 외형을 보려면 Fusion 360 원본 설계에서 STL ZIP을 다시 내보내야 합니다.

## 0. 이 브랜치의 현재 상태

| 단계 | 상태 |
|---|---|
| Fusion 모델로부터 확인된 37개 Occurrence / 36개 As-Built Revolute Joint | 분석 기록 있음 |
| Rigid Group 1개, 8개 폐루프 / 12개 예상 구동 자유도 | 기구학 해석 완료 |
| MuJoCo 고정 몸체 + 임시 관절 도식 생성 | 코드 제공 |
| 97프레임 폐루프 구속 기구학 재생성 | 코드 제공 |
| Fusion STL 37개를 외형으로 붙이기 | 변환 코드 제공, **사용자 실제 STL로 미검증** |
| MuJoCo 중력·접촉·질량·서보 물리·보행 RL | **미구현** |

사용할 컴퓨터: Windows + Python 3.12 권장. 첫 설치에는 인터넷 필요.
실제 로봇 개, Raspberry Pi, Hailo와 연결하지 않습니다.

## 1. GitHub에서 코드 한 번에 받기

1. 저장소 **GUNOBAE/quadruped_robot_bgh**의 브랜치를
   **feature/mujoco-cad-preview**로 선택합니다.
2. 상단 **Code → Download ZIP**을 누릅니다.
3. ZIP을 **완전히 압축 해제**합니다 (ZIP 내부에서 CMD 실행 금지).
4. 압축 푼 폴더 안의 **mujoco_cad_preview** 폴더를 엽니다.
5. **NEW_PC_START.cmd**를 더블클릭합니다.

첫 실행은 Python 가상 환경을 만들고 MuJoCo/NumPy/SciPy를 설치한 뒤,
아래 데이터를 **자동 생성**합니다.

    mujoco_cad_preview/
      NEW_PC_START.cmd               <- 새 PC에서 먼저 실행!
      requirements_new_pc.txt
      make_seed.py                    <- 이전 관절 좌표 축약본 복구
      build_preview.py                <- 관절/폐루프 MJCF 생성
      solve_motion.py                 <- 97프레임 작은 움직임 재계산
      view_preview.py                 <- 도식(막대) 미리보기 / 검사
      attach_cad_meshes.py            <- 실제 STL 외형 부착
      view_cad_preview.py             <- 실제 CAD 외형 보기
      fusion_export_meshes.py         <- Fusion에서 실행하는 STL 추출용
      Quadruped_CAD_Meshes.zip        <- 아직 없으면 사용자가 Fusion에서 생성
      output/                         <- CMD 실행 시 자동 생성
        quadruped_preview.xml
        model_manifest.json
        preview_motion.json
        replay_joint_names.json
        kinematic_validation.json

**중요:** CAD ZIP을 넣기 전에는 이전처럼 실제 CAD 외형이 아니라
단순 링크 도식만 생성됩니다. 코드는 자동 설치되지만 CAD 원본은
저장소에 들어있지 않기 때문입니다.

## 2. 실제 Fusion 모델 외형을 보려면

이미 Windows PC에서 **Quadruped_CAD_Meshes.zip**을 갖고 있다면
그 ZIP을 **mujoco_cad_preview 폴더에 그대로 복사**하세요.
다시 **NEW_PC_START.cmd**를 실행하면 STL 외형을 적용하려 시도합니다.

원본 CAD ZIP이 없다면 다음 방법으로 새로 만드세요.

1. 새 PC의 **Fusion 360**을 실행하고 Fusion 클라우드 또는 로컬에서
   기존 **ASSAMBLE_A.f3d** 설계를 엽니다.
2. Fusion → **Utilities → Scripts and Add-Ins → Create Script**
   - Type: Script
   - Name: QuadrupedMeshExport
   - Language: Python
3. 생성된 Python 스크립트를 VS Code에서 엽니다.
4. 이 폴더의 **fusion_export_meshes.py** 코드를 **전부 복사**해서
   새 Fusion 스크립트 내용을 교체하고 Ctrl+S로 저장합니다.
5. **Fusion 360으로 돌아가서** 스크립트를 Run 합니다.
   (VS Code 우측 실행 버튼이 아닙니다.)
6. 다운로드 폴더에 생성된 **Quadruped_CAD_Meshes.zip**을
   GitHub에서 받은 **mujoco_cad_preview** 폴더로 복사합니다.
7. **NEW_PC_START.cmd**를 다시 실행합니다.

성공하면:
- output/cad_preview.xml
- output/cad_fit_report.json
- output/cad_meshes/ (좌표 보정된 STL 37개)

생성되고 MuJoCo CAD Viewer가 열립니다.
**그림은 CAD 원본을 반영하더라도 아직 실물 보행 학습 모델은 아닙니다.**

## 3. 다운로드 없이 Git을 쓰는 방법 (선택)

Git이 설치되어 있다면 PowerShell에서:

    git clone -b feature/mujoco-cad-preview --single-branch https://github.com/GUNOBAE/quadruped_robot_bgh.git
    cd quadruped_robot_bgh\mujoco_cad_preview
    .\NEW_PC_START.cmd

**기존 main을 업데이트하거나 PR을 병합할 필요는 없습니다.**
이 기능은 PR #1의 작업 브랜치에 있습니다.

## 4. 오류 해결

- **'py' is not recognized**: Windows Python 3.12 설치 후 다시 실행.
- **Fusion CAD ZIP missing**: 위 '2. 실제 Fusion 모델 외형' 절대로 STL 내보내기.
- **Mesh bounds mismatch**: 실행을 중단하는 것이 정상입니다.
  output/cad_fit_report.json에서 부품별 좌표·단위 차이를 확인하세요.
- **MuJoCo ParseXML Error opening file**: 한글 경로 대응을 위해
  XML을 Python에서 읽어 문자열로 전달하고,
  STL도 MuJoCo MjVfs로 로딩하도록 구현했습니다.
  그 외 오류는 전체 콘솔 메시지를 보관하세요.
- **로봇 외형이 네 Fusion 모델과 다름**: 실제 STL ZIP을 넣었는지 확인.
  STL이 없으면 기본 도식 미리보기만 표시됩니다.

비 GUI 검사:

    .venv\Scripts\python.exe view_preview.py --verify-only
    .venv\Scripts\python.exe view_cad_preview.py --verify-only

## 5. 꼭 알아야 하는 한계

- 참조 관절 좌표는 과거 진단 JSON에서 재구성한 축약본입니다.
  원본 전체 Fusion 물성/회전범위/정확한 발 접촉 위치를 대체하지 않습니다.
- 36개 회전 관절이 모두 구동되는 것은 아닙니다.
  폐루프가 포함되어 **12개의 독립 운동 자유도**로 해석됩니다.
- 12개 제어 입력은 **시험용 관절**, 실제 서보 채널과 일치한다고 보장하지 않습니다.
- 원래 CAD 추정 질량 합계(약 7.16 kg)는 실물 무게와 다를 수 있습니다.
- 고정 몸체, 중력·접촉 비활성화, 임시 관성값. 따라서
  실제 무게를 지탱하거나 서고 걷는 검증은 전혀 하지 않았습니다.
- 원래 Fusion 설계는 이 코드로 변경하지 않습니다.

## 6. 다음 연구 순서

실제 CAD 외형·관절을 표시한 뒤:
1. 외형이 틀어지지 않는지, 연결봉 폐루프가 유지되는지 확인.
2. 서보 중립각과 가동범위 및 실제 발바닥 위치 확정.
3. 실측 총질량과 관성, 발 접촉 형상, 모터 토크/속도 반영.
4. freejoint / gravity / contact를 켜고 안정적인 Standing.
5. 기본 저속 Gait 검증 후, MuJoCo PPO 강화학습 진행.
6. 마지막에 Pi 5/실물 서보에 정책을 옮기고 안전 제어.

**GitHub PR #1은 아직 Draft**입니다. 코드가 새 PC에서 실행되는지
실기 검증한 뒤에만 병합을 검토하세요.
