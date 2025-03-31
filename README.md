# Docker 설치
## Docker CE(Community Edition) 설치를 추천
```sh
$ apt-get update
$ sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
$ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io
```
## Docker compose plugin 설치
```sh
$ sudo apt-get install docker-compose-plugin
```

## Add user to docker group
```sh
$ sudo groupadd -f docker
$ sudo usermod -aG docker <UserName>
```

# Docker Image Build
## Publisher build
```sh
docker build --tag pub:aimd -f ./Dockerfile_pub .
```

### Subscriber build
```sh
docker build --tag sub:aimd -f ./Dockerfile_sub .
```

# Run with docker compose

```sh
$ cat docker-compose.yml
```
```
version: '3.8'
services:
  ros-listener:
    image: "sub:aimd"
    container_name: ros-listener
    network_mode: "host"
    tty: true
#    volumes: 
#       - ./data:/data
  ros-talker:
    image: "pub:aimd"
    container_name: ros-talker
    network_mode: "host"
    tty: true
```

``` sh
$ docker compose up
```


# ROS Setup 과정 정리 (py_pubsub sample 생성 과정) 
## Create a ROS workspace
``` sh
mkdir -p ros2_ws/src
cd ros2_ws/src
```

## Create a package
``` sh
ros2 pkg create --build-type ament_python py_pubsub
```

### Download sample codes
``` sh
cd ros2_ws/src/py_pubsub/py_pubsub
```

### Download publisher
``` sh
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
### Download subscriber
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

### Modify setup.py to add talker & listener to console_scripts
    entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
    },


## Build and run
``` sh
cd ros2_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select py_pubsub
```

## source setup.bash (local)
``` sh
source install/setup.bash
```


# 참고
## colcon build 중 에러가 발생하는 경우
``` sh
pip uninstall empy
pip install empy==3.3.4

pip install catkin_pkg
pip install numpy
pip install lark
```
