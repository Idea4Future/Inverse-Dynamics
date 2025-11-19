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


---

## 2. 파이썬 실습 코드 실행 방법

### 2.1 기본 실행

1. **터미널 실행**

   Ubuntu에서 터미널을 엽니다.

2. **저장소 폴더로 이동**

   ```bash
   cd ~/path/to/demo_code.py
   ```

3. **실습 코드 실행**

   ```bash
   python3 demo_code.py
   ```

4. **실행 결과**

   코드 실행이 끝나면 다음과 같은 파일들이 `outputs/` 폴더에 생성됩니다.

   - `tsid_qp_control.gif` : 2-자유도 로봇팔이 궤적을 추종하는 애니메이션
   - `tsid_qp_demo_plots.png` : 조인트 `q`, `dq`, `ddq`, 토크, QP 솔버 상태 등의 시간 그래프
   - `tsid_qp_demo_plots_explain.png` : 트래킹 에러, EE 궤적, 최종 포즈 등을 설명하는 플롯

<br/>

### 2.2 MP4 영상 저장을 위한 ffmpeg 설치 (선택 사항)

코드를 실행했을 때 예를 들어 다음과 같은 출력이 보일 수 있습니다.

```text
Saved GIF: outputs/tsid_qp_control.gif
MP4 save failed (need ffmpeg?): [WinError 2] 지정된 파일을 찾을 수 없습니다
```

이는 **GIF 파일은 저장되었지만, MP4 파일을 저장하려면 `ffmpeg`가 필요하다**는 의미입니다.

Ubuntu에서 ffmpeg를 설치하려면 다음 명령을 실행합니다.

```bash
sudo apt update
sudo apt install ffmpeg
```

설치 후 다시 코드를 실행합니다.

```bash
python3 demo_code.py
```

이제 `outputs/` 폴더에 GIF와 함께 MP4 파일도 저장되도록 설정할 수 있습니다.  

---

## 3. 데모 결과 예시 (GIF & 이미지)

아래는 실습 코드를 실행했을 때 얻을 수 있는 결과 예시입니다.  

![tsid_qp_control](https://github.com/user-attachments/assets/f77ea7d5-6573-4a2f-b5bd-46b27557da3d)<img width="1500" height="1800" alt="tsid_qp_demo_plots" src="https://github.com/user-attachments/assets/a35985f8-d1db-456a-855a-c1eb11ff7ba2" /><img width="1800" height="750" alt="tsid_qp_demo_plots_explain" src="https://github.com/user-attachments/assets/b1166664-72af-4d0d-a65c-d207480052ae" />
