# Camera and LiDAR module (triggered)
### 0. Functionalities of this 'camlidar_module' ROS project
* Two sensor topics (camera and LiDAR pcl) are time-synchronized by the Arduino Mega2560 hardware. Arduino Mega2560 fires two trigger signals: (1) 20 Hz digital trigger signals (5V high) with 300 us width for 'camera trigger', (2) 1 Hz PPS signal for Velodyne interface box. (3) Arduino Mega2560 also generates a simulated NMEA ($GPRMC) message carrying via the Serial1 port (TX1 pin). This NMEA message carries timestamp (which is firing time of PPS signal measured by Arduino clock) as a form of 'hhmmss.sss' (upto milliseconds resolution).
* Subscribes sensor_msgs::Image ( /0/image_raw ) and pcl::PointCloud2 (/lidar0/velodyne_points ) in function 'callbackImageLidarSync()'
* undistorts RGB image by using calibration parameters from the '*.yaml' file in function 'undistortCurrentImage()' <-- this function is executed in the 'callbackImageLidarSync()'
* saves snapshots from streaming 
* TODO: warp LiDAR 3D points and 

### 1. How to install?
```
 cd ~/catkin_ws/src
 git clone https://github.com/ChanghyeonKim93/camlidar_module.git 
```
#### 1.1. Prerequisites before building this 'camlidar_module' repository
##### [1] 'ChanghyeonKim93/bluefox_ros'
In your 'catkin_ws/src',
```
 git clone https://github.com/ChanghyeonKim93/bluefox_ros.git 
```
##### [2] 'rosserial*' (all rosserial libraries)
```
 sudo apt-get install ros-{DISTRO}-rosserial*
```
##### [3] Matrix Vision mvBLueFOX2-MLC camera driver (Installation files are included in 'camera_drivers' folder in this repository)
 센서 하드웨어의 camera는 독일 matrixvision (baluff) 사의 mvBlueFOX2-MLC200wC 제품을 사용한다 (color). 사용하기 위해서는 독립적인 library를 설치해주어야 한다. 본 문서와 같은 폴더의 ‘tools’ 폴더의 ‘matrixvision.zip’ 압축을 풀고, 압축이 풀린 경로로 가서 라이브러리 설치를 위해 아래 명령을 순차적으로 실행한다.
 ```
   sudo chmod +x install_mvGenTL_Acquire.sh 
   sudo ./install_mvGenTL_Acquire.sh 
```
 설치 시, 모두 yes를 입력하여 설치를 완료한다. 조금 걸린다. 
 blueFOX-MLC 카메라의 경우, GenTL을 쓰지않고 hardware specified library를 쓴다. 따라서 blueFOX 카메라 사용할 때에는 전용 드라이버도 함께 설치해주자. 설치 방법은 거의 동일하다!

* libusb 접근 권한 부여 (for blueFOX camera only)

 blueFOX 카메라는 /dev/usb 쪽에 읽기+쓰기 모두 접근하는데, linux 기본 세팅은 모든 유저에게 접근 권한이 주어져있지 않다. 따라서, 모든 유저가 dev 쪽을 건드릴 수 있도록 접근 권한을 줘야한다.

vectornav 사 vn-100t 사용 시 간혹 /imu: VN: Permission denied 가 뜰 수 있다. 이는 vn-100t 가 ttyUSB0 을 사용하는데 ttyUSB0은 root 와 dialout Group에만 읽기/쓰기 권한이 주어져서 이다. 현재 Linux user의 권한이 dialout Group 안에 들어가있는지 확인하기 

command 에서
```
   id
```
dialout 이라는 글자가 안보이면 그 그룹에 안들어가있는것.
>> sudo usermod -a -G dialout <username> 
으로 권한을 주고, 꼭 REBOOT을 해야지 권한이 주어진다.
 
 설치 완료 후, 재부팅을 해준다. 재부팅 시에 library가 설치되어있는 경로에 대한 환경변수가 설정 되는 것으로 보인다.

##### [4] Velodyne LiDAR driver
 velodyne lidar의 데이터를 ROS topic으로 출력하는 ROS package를 설치해야한다. 아래 주소에서 설치 방법이 상세하게 설명되어있으며, 해당 내용을 그대로 따라서 설치하면 된다.
(http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
 설치 후, 실행이 되는지 확인해보자. 단, 본 lidar+camera 하드웨어 세팅에서는 lidar 의 IP를 static IP로 설정해주었기 때문에, roscd velodyne_pointcloud 에서 /launch 폴더 내 VLP16_points.launch 파일을 수정해야 사용 할 수 있다. 
 연구실 내 각 LiDAR에는 고정 IP와 port 번호 규칙을 정하였다. (roscore가 구동되는 laptop ip: 192.168.1.1)
```
  lidar0: 192.168.1.101 (port 2357)
 	lidar1: 192.168.1.201 (port 2358)
```
 위 IP와 port 번호를 참고하여 launch 파일을 수정해주자.
```
 <arg name=”device_ip” default=”” /> <arg name=”device_ip” default=”192.168.1.xxx” />
 <arg name=”port” default=”” /> <arg name=”port” default=”2357” />
```
 이후, 아래 명령을 실행하여 rviz를 통해 pointcloud2 토픽이 잘 나오는지 확인해본다.
```
   roslaunch velodyne_pointcloud VLP16_points.launch
```

### 2. Build
``` 
 cd ~/catkin_ws
 catkin_make
```

### 3. Run
```
 roslaunch camlidar_module camlidar_sync.launch
```

### 4. How to integrate this module into your ROS code
* In 'main.cpp' 