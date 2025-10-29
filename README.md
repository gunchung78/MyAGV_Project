# MyAGV_Project

실행 방법

0) 전제

    OS: Ubuntu 20.04

    ROS: Noetic (Python3)

1) apt-get 갱신
   
    sudo apt-get update
   
    sudo apt-get install -y pkg-config


3) 의존성 팩키지 자동 설정

    chmod +x scripts/bootstrap.sh scripts/build.sh 2>/dev/null || true
    
    cd ~/MyAGV_Project/myagv_ros
    
    bash ./scripts/bootstrap.sh
    
    source /opt/ros/noetic/setup.bash

3) 빌드 진행

    catkin_make
    
    source devel/setup.bash 

4) 최종 실행

    python3 myagv_operation.py

터미널에서 연결 시도시 성공적으로 접근되면 세팅 완료
