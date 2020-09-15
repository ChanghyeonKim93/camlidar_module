# Camera and LiDAR module (triggered)
## How to install?
* **camera driver
 센서 하드웨어의 camera는 독일 matrixvision (baluff) 사의 mvBlueCOUGAR-X-104i 제품을 사용한다 (grayscale). 해당 제품은 산업용 이미지솔루션의 통신 표준으로 자리잡은 GigE (이미지 전송용 LAN 통신 protocol) 을 이용하며, 사용하기 위해서는 독립적인 library를 설치해주어야 한다.
 본 문서와 같은 폴더의 ‘tools’ 폴더의 ‘matrixvision.zip’ 압축을 풀고, 압축이 풀린 경로로 가서 라이브러리 설치를 위해 아래 명령을 순차적으로 실행한다.
 
>> sudo chmod +x install_mvGenTL_Acquire.sh 
>> sudo ./install_mvGenTL_Acquire.sh 

 설치 시, 모두 yes를 입력하여 설치를 완료한다. 조금 걸린다. 
 설치 완료 후, 재부팅을 해준다. 재부팅 시에 library가 설치되어있는 경로에 대한 환경변수가 설정 되는 것으로 보인다.
 blueFOX-MLC 카메라의 경우, GenTL을 쓰지않고 hardware specified library를 쓴다. 따라서bluefox 카메라 사용할 때에는 전용 드라이버도 함께 설치해주자. 설치 방법은 거의 동일하다!

* *libusb 접근 권한 부여 (for blueFOX camera only)

 blueFOX 카메라는 /dev/usb 쪽에 읽기+쓰기 모두 접근하는데, linux 기본 세팅은 모든 유저에게 접근 권한이 주어져있지 않다. 따라서, 모든 유저가 dev 쪽을 건드릴 수 있도록 접근 권한을 줘야한다.

vectornav 사 vn-100t 사용 시 간혹 /imu: VN: Permission denied 가 뜰 수 있다. 이는 vn-100t 가 ttyUSB0 을 사용하는데 ttyUSB0은 root 와 dialout Group에만 읽기/쓰기 권한이 주어져서 이다. 현재 Linux user의 권한이 dialout Group 안에 들어가있는지 확인하기 

command 에서
>>id
dialout 이라는 글자가 안보이면 그 그룹에 안들어가있는것.
>> sudo usermod -a -G dialout <username> 
으로 권한을 주고, 꼭 REBOOT을 해야지 권한이 주어진다.

## LiDAR driver**
 velodyne lidar의 데이터를 ROS topic으로 출력하는 rospackage를 설치해야한다. 아래 주소에서 설치 방법이 상세하게 설명되어있다.
(http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
 설치 후, 실행이 되는지 확인해보자. 단, 본 lidar+camera 하드웨어 세팅에서는 lidar 의 IP를 static IP로 설정해주었기 때문에, roscd velodyne_pointcloud 에서 /launch 폴더 내 VLP16_points.launch 파일을 수정해야 사용 할 수 있다. 
 연구실 내 각 LiDAR에는 고정 IP와 port 번호를 부여 해두었다.

  	lidar0: 192.168.1.101 (port 2357)
 	lidar1: 192.168.1.201 (port 2358)
 
 위 IP와 port 번호를 참고하여 launch 파일을 수정해주자.

 <arg name=”device_ip” default=”” />  <arg name=”device_ip” default=”192.168.1.xxx” />
 <arg name=”port” default=”” />  <arg name=”port” default=”2357” />

 이후, 아래 명령을 실행하여 rviz를 통해 pointcloud2 토픽이 잘 나오는지 확인해본다.

>> roslaunch velodyne_pointcloud VLP16_points.launch
