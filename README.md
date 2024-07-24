
# 2024 bitdol <bird alert project> V1.2

## 프로젝트 설명
이 레포지토리는 팀원분들이 새로 작성하신 패키지들을 업로드 하는 레포지토리입니다.  
본 레포지토리는 `catkin_ws/src` 에 `bird_alert`라는 하위 폴더입니다.
따라서 우리가 작성하는 모든 프로젝트 코드는 `catkin_ws/src/bird_alert`에서만 작성됩니다.

## 사전 지식

명확한 분업을 위하여 지정된 한 폴더 내에서만 작성하는 것을 권장합니다.  
만약 여러개의 패키지를 만들고 싶다 하면 다음과 같은 구조를 따르세요:

```
catkin_ws/
├── src/
│   ├── bird_alert/
│   │   ├── bird_camera/
│   │   │   ├──bird_detection/
│   │   │   └──camera_launch/
│   │   ├── bird_turret/
│   │   └── bird_autodriving/
│   └── turtlebot3/
└── CMakeLists.txt
```

이런식으로 제가 지정해준 폴더 내에 여러 패키지를 만들어주시면 됩니다.

## 레포지토리 지정폴더
이 레포지토리에 지정된 폴더 내에 여러분들이 작성하신 패키지들을 저장해주시길 바랍니다.
- **규리**: `/bird_camera`
- **승헌 기웅**: `/bird_turret`
- **상훈 재웅**: `/bird_autodriving`

이 폴더 내부에 패키지를 넣어주시면 됩니다. 그런 후에 지정폴더를 공유해주시면 됩니다.

### 레포지토리 클론 및 워크스페이스 설정 방법

1. `catkin_ws/src` 디렉토리로 이동합니다.
   ```sh
   cd ~/catkin_ws/src
   ```

2. 레포지토리를 클론합니다.
   ```sh
   git clone <레포지토리 URL>
   ```

3. `catkin_ws` 디렉토리로 이동합니다.
   ```sh
   cd ~/catkin_ws
   ```

4. `catkin_ws` 디렉토리에서 catkin 워크스페이스를 빌드합니다. (새로운 패키지를 다운받으면 이를 실행시키기 위하여 빌드를 해주어야합니다.)
   ```sh
   catkin_make
   ```

5. 환경 설정을 적용합니다. (여러분의 PC에 이 명령어가 bashrc에 추가되어 있을 것이기 때문에 `source ~/.bashrc` 해주어도 됩니다.)
   ```sh
   source devel/setup.bash
   ```

### ROS 패키지 실행

원하는 패키지를 실행하려면 다음과 같이 합니다:

```sh
roslaunch <패키지명> <launch파일명>.launch
```

예시:
```sh
roslaunch bird_detection my_bird_detection.launch
```
(이 런치파일 안에는 파이카메라를 키는 런치파일과, 새를 인식시켜서 메인코드로 새의 좌표를 보내는 py코드의 노드가 포함되어있음)

##  **프로젝트를 처음 시작할때**
1. 깃허브의 코드를 로컬로 다운로드
   ```sh
   git clone https://github.com/hun7176/bird_alert.git
   ```
   목적: 원격 저장소(깃허브)의 전체 내용을 로컬 컴퓨터(내 컴퓨터)에 복제합니다.  
   사용 시점: 프로젝트를 처음 시작할 때, 또는 새로운 작업 환경을 설정할 때 사용합니다. 즉, 원격 저장소의 복사본을 처음부터 끝까지 로컬로 가져옵니다.  
   작동 방식: 원격 저장소의 모든 파일, 브랜치, 커밋 기록 등을 로컬 저장소로 복제합니다. 로컬 저장소는 원격 저장소와 동일한 상태로 시작됩니다.

2. **로컬의 수정된 코드를 깃허브로 업로드**  

   a. 컴퓨터에 깃허브 계정을 등록해야합니다.
   ```sh
   $ git config --global user.name "seungH"
   $ git config --global user.email hun7176@naver.com
   ```
   b. 작업한 디렉토리로 이동하여 변경된 파일을 스테이징합니다.
   ```sh
   cd ~/catkin_ws/src/bird_alert
   git add .
   ```
   c. **커밋 생성하기**  
   변경 사항에 대한 설명을 포함하여 커밋을 생성합니다. (커밋이란 댓글 같은 것. 보통 버전명이나 날짜를 기입)
   ```sh
   git commit -m "작업한 내용에 대한 설명"
   ```

   c. **변경 사항 푸시하기**  
   원격 저장소에 변경 사항을 푸시합니다.
   ```sh
   git push origin main
   ```

   ***푸시 에러 발생 시***  
   만약 충돌이나 에러가 발생하면, 최신 변경 사항을 먼저 받아와야 합니다.
   ```sh
   git pull origin main
   ```
   이후 변경 사항을 다시 푸시합니다.
   ```sh
   git push origin main
   ```


##  **프로젝트를 진행중일때(위에 셋업을 한번이라도 한 상태)**
   **1.이미 한 번 git clone을 한 상태여서 내 컴퓨터에 폴더들이 이미 존재할 때**
   ```sh
   git pull origin main
   ```
   목적: 원격 저장소의 변경 사항을 로컬 저장소에 가져와 병합합니다.  
   사용 시점: 이미 복제한 저장소에서 로컬 작업을 계속하고 있을 때, 원격 저장소에 새로 추가된 변경 사항을 가져와 로컬 저장소와 동기화할 때 사용합니다.  
   작동 방식: 원격 저장소의 지정된 브랜치에서 최신 커밋을 가져와 현재 체크아웃된 로컬 브랜치에 병합합니다. 이 과정에서 fetch와 merge를 자동으로 수행합니다.

   **2. 코드를 모두 작성한 후 깃허브에 업로드하는 방법**

   a. **변경 사항 추가하기**  
   작업한 디렉토리로 이동하여 변경된 파일을 스테이징합니다.
   ```sh
   cd ~/catkin_ws/src/bird_alert
   git add .
   ```

   b. **커밋 생성하기**  
   변경 사항에 대한 설명을 포함하여 커밋을 생성합니다. (커밋이란 댓글 같은 것. 보통 버전명이나 날짜를 기입)
   ```sh
   git commit -m "작업한 내용에 대한 설명"
   ```
   c. **변경 사항 푸시하기**  
   원격 저장소에 변경 사항을 푸시합니다.
   ```sh
   git push origin main
   ```


## 예제

1. 작업한 디렉토리로 이동
   ```sh
   cd ~/catkin_ws/src/bird_alert
   ```

2. 변경된 파일 스테이징
   ```sh
   git add .
   ```

3. 커밋 생성
   ```sh
   git commit -m "7월 24일 디텍션 코드 수정본"
   ```

4. 원격 저장소에 변경 사항 푸시
   ```sh
   git push origin main
   ```

### 깃허브 사용 시 주의사항

- 작업 시작 전 항상 최신 상태로 유지하기 위해 pull 명령어를 사용하여 원격 저장소의 최신 변경 사항을 가져옵니다.
   ```sh
   git pull origin main
   ```

- 커밋 메시지는 작업 내용을 명확히 알 수 있도록 작성합니다.

- 충돌이 발생하면 신속하게 해결하여 팀원들과의 작업에 지장을 주지 않도록 합니다.

## 마무리

이제 깃허브를 사용하여 협업할 준비가 되었습니다.

## 필수 추가사항

1. 협업을 하는데 다른 분야의 작업 폴더를 잘못 수정하면 안됩니다! 잘못 수정한 다른 사람의 폴더가 덮어쓰기 되어서 날라가버릴 수 있습니다.

2. `.gitignore` 파일 내에 업로드 하지 않을 폴더를 추가해 주어도 됩니다.
   ```sh
   cd ~/catkin_ws/src
   nano .gitignore
   ```

   예시:
   ```sh
   DynamixelSDK/
   turtlebot3/
   turtlebot3_msgs/
   ld08_driver/
   turtlebot3_autorace_2020/
   ```

