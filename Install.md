# Install
## 의존성
* Ubuntu 22.04
* ROS 2 Humble


## network 설정
* 고정 IP, DNS,

## 설치
```
cd ~
git clone https://github.com/badaproject/bada2_ws
cd ~/bada2_ws
colcon build

source ~/bada2_ws/install/setup.bash
```

## 부팅 서비스 등록
* 부팅시 자동으로 ros 실행되도록 등록
```

```
* /home/ubuntu/ros2_start.sh
```
#!/bin/bash
# ROS 2 환경 설정
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

# Launch 파일 실행
exec ros2 launch bada_bringup bada.launch.py
```

* /etc/systemd/system/bada.service
```
[Unit]
Description=ROS2 Bada Bringup
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu
ExecStart=/home/ubuntu/ros2_start.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target

```

* service 등록 및 실행
```
sudo systemctl daemon-reload
sudo systemctl enable bada.service
sudo systemctl start bada.service
```

