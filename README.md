# Inverse-Dynamics
Inverse-Dynamics Tutorial Lecture

<br/>

---
## 1. 강의 링크 및 구성

전체 강의 재생목록은 아래 유튜브 링크에서 볼 수 있습니다.

- 🔗 **강의 재생목록:**  
  https://youtube.com/playlist?list=PLDLZGCGcAQ5xPP4mWTmy94sIaaRITggly&si=-kCAALn_y7AZifZd

강의는 다음과 같이 구성됩니다.

- **1 ~ 4강: 개념 강의**
  - 로봇 운동방정식 소개
  - TSID 기본 개념 소개
  - QP 기반 제어 문제 정식화
  - 궤적 추종, 오차 동역학, 제약 조건 등 이론 설명
  - 👉 이 저장소의 `강의자료/` 폴더에 있는 PDF 슬라이드를 참고하면 됩니다.

- **5강: 파이썬 실습 강의**
  - 2-자유도 로봇팔 모델 정의
  - TSID / QP 제어식 구현
  - 👉 이 저장소의 `파이썬_실습_코드/` 폴더에 있는 파이썬 실습 코드를 사용합니다.
 
- **6강: Mujoco 실습 강의**
  - mujoco를 활용한 C++ 기반 panda/ur5 로봇 제어
  - TSID / QP 제어식 구현

---
## 2. 파이썬 실습 강의 결과 (GIF & 이미지)

아래는 파이썬 실습 코드를 실행했을 때 얻을 수 있는 결과입니다.  

<table>
  <tr>
    <td align="left">
      <img src="https://github.com/user-attachments/assets/f77ea7d5-6573-4a2f-b5bd-46b27557da3d"
           alt="tsid_qp_control"
           width="250">
    </td>
    <td align="center">
      <img src="https://github.com/user-attachments/assets/a35985f8-d1db-456a-855a-c1eb11ff7ba2"
           alt="tsid_qp_demo_plots"
           width="400">
    </td>
    <td align="right">
      <img src="https://github.com/user-attachments/assets/b1166664-72af-4d0d-a65c-d207480052ae"
           alt="tsid_qp_demo_plots_explain"
           width="400">
    </td>
  </tr>
</table>

---

## 3. Mujoco 실습 강의 결과

아래는 Mujoco 실습 코드를 실행했을 때 얻을 수 있는 결과입니다. 


---
## 4. Reference
- 해당 튜토리얼 강의는 https://github.com/stack-of-tasks/tsid 자료와 강의를 참고하여 학습용으로 제작되었습니다.
- Qontrol 실습 : https://gitlab.inria.fr/auctus-team/components/control/qontrol
- (https://auctus-team.gitlabpages.inria.fr/components/control/qontrol/index.html)

