# Fusion 360 CAD mesh -> MuJoCo linkage preview

이 폴더는 **기존 BGH 로봇개 MuJoCo V0.1 미리보기**에 Fusion 360에서
내보낸 37개 STL 형상을 붙이는 도구입니다.

> 이 버전은 CAD **외형 + 관절 기구학 미리보기 전용**입니다.
> 중력/발 접촉/서보 토크/질량 보정/보행 강화학습은 구현되지 않았습니다.
> 실제 로봇을 제어하는 코드는 포함되어 있지 않습니다.

## 필요한 기존 파일

사용자 PC에서 **이전에 정상 실행했던 quadruped_v01 폴더**가 있어야 합니다.
그 안에는 다음 파일이 이미 있어야 합니다.

- RUN_PREVIEW.cmd
- audit_excerpt_reference.json
- requirements.txt
- output/quadruped_preview.xml
- output/model_manifest.json
- output/preview_motion.json
- output/replay_joint_names.json

이 GitHub 브랜치에는 이전 임시 MuJoCo 프로젝트 전체나 **사용자의 원본 CAD/STL
37개**를 공개 업로드하지 않았습니다. 개인 제작 STL ZIP은 사용자 PC에 보관됩니다.

## 사용법 (Windows)

1. GitHub에서 이 브랜치의 세 파일을 받습니다.
   - attach_cad_meshes.py
   - view_cad_preview.py
   - RUN_CAD_PREVIEW.cmd
2. 세 파일을 **기존 quadruped_v01 폴더의 최상위**에 복사합니다.
   별도의 폴더에 놓으면 기존 관절 파일을 찾지 못합니다.
3. Fusion 360에서 내보냈던 원본 **Quadruped_CAD_Meshes.zip**을
   같은 quadruped_v01 폴더에 놓습니다. ZIP을 풀 필요는 없습니다.
4. RUN_CAD_PREVIEW.cmd를 더블클릭합니다.
5. 성공하면 MuJoCo에 실제 STL 외형이 표시되고 기존 97프레임
   미리보기 움직임이 재생됩니다.

경로 예시:

    C:\RobotDog\quadruped_v01\
      RUN_PREVIEW.cmd
      RUN_CAD_PREVIEW.cmd                <- 이 브랜치
      attach_cad_meshes.py              <- 이 브랜치
      view_cad_preview.py               <- 이 브랜치
      Quadruped_CAD_Meshes.zip          <- Fusion 360에서 내보낸 ZIP
      audit_excerpt_reference.json
      output\
        quadruped_preview.xml
        model_manifest.json
        preview_motion.json
        replay_joint_names.json
        cad_preview.xml                 <- 실행 시 생성
        cad_fit_report.json             <- 실행 시 생성
        cad_meshes\                     <- 실행 시 생성

## 오류가 나는 경우

- "Missing output\\quadruped_preview.xml" : 기존 quadruped_v01 폴더가 아닌
  위치에서 실행한 것. **이 브랜치의 3개 파일을 기존 폴더로 옮겨주세요.**
- "Missing Quadruped_CAD_Meshes.zip" : 원본 STL ZIP을 같은 폴더에 넣습니다.
- "Mesh bounds mismatch" : 자동 맞춤이 실패한 부품이 있다는 뜻.
  **output/cad_fit_report.json**을 보내주세요. 좌표와 단위 검사 결과가 기록됩니다.
- Python 미설치/패키지 오류: Windows Python launcher (py)가 필요합니다.
  MuJoCo + NumPy는 첫 실행 시 가상환경에 설치됩니다.
- 한글 경로 문제: XML과 STL을 Python으로 읽어 메모리로 전달하는 방식으로
  우회했지만, 다른 OS 환경 문제는 발생할 수 있습니다.

직접 검사만 하고 창을 열지 않으려면 quadruped_v01 폴더에서:

    .venv\Scripts\python.exe attach_cad_meshes.py
    .venv\Scripts\python.exe view_cad_preview.py --verify-only

## 변환 방식과 주의 사항

- ZIP의 mesh_manifest.json / STL 37개를 확인합니다.
- Fusion bounding box와 STL bounds를 비교해 mm/cm/m/inch 및
  assembly/component 좌표계 중 가장 일치하는 조합을 선택합니다.
- 일치 정도가 낮으면 **변환을 중단**하고 보고서를 작성합니다.
- 기존 관절/폐루프 MJCF는 수정하지 않습니다.
- 기존 primitive geoms는 보이지 않도록 한 뒤 group=1 시각화
  mesh를 추가합니다. 실제 충돌과 질량은 여전히 **임시값**입니다.
- 현재 원본 CAD ZIP으로의 변환 성공은 **아직 실기 검증 전**입니다.
  합성 STL 37개 테스트로 importer 작업 흐름만 확인했습니다.
- 실제 보행 학습을 하려면 질량·관성·발 접촉·중력·서보 제한·구동
  관절 지정 및 베이스 자유도를 별도로 모델링해야 합니다.

## 원본 자료

- 프로젝트: GUNOBAE/quadruped_robot_bgh
- Fusion 원본: ASSAMBLE_A.f3d (사용자 보관)
- 진단 JSON: quadruped_joint_audit.json (사용자 보관)
- STL ZIP: Quadruped_CAD_Meshes.zip (사용자 보관)
