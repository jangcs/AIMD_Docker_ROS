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
$ cd Publisher
```

```sh
$ cat Dockerfile
```
```
FROM python:3.8

RUN pip3 install paho-mqtt
COPY publish.py publish.py

CMD python3 publish.py
```

```sh
$ docker build -t mqtt-publish:v1 .
```

```sh
$ docker images
```

### Subscriber build
```sh
$ cd Subscriber
```

```sh
$ cat Dockerfile_pub
```

```
FROM nvidia/cuda:12.2.2-base-ubuntu20.04

...
RUN apt-get install -y ros-foxy-desktop
...


```
```sh
$ docker build -t mqtt-subscribe:v1 .
```

```sh
$ docker images
```

# Run with docker compose

```sh
$ cd PubSub
```

```sh
$ cat docker-compose.yml
```
```
version: '3.8'
services:
  mqtt-subscribe:
    image: "mqtt-subscribe:v1"
    container_name: mqtt-sub
    network_mode: "host"
    tty: true
#    volumes: 
#       - ./data:/data
  mqtt-publish:
    image: "mqtt-publish:v1"
    container_name: mqtt-pub
    network_mode: "host"
    tty: true
```

``` sh
$ docker compose up
```



pip uninstall empy
pip install empy==3.3.4

pip install catkin_pkg
pip install numpy
pip install lark

# ROS Setup
## Create a ROS workspace
mkdir -p ros2_ws/src
cd ros2_ws/src

## Create a package
ros2 pkg create --build-type ament_python py_pubsub

### Download sample codes
cd ros2_ws/src/py_pubsub/py_pubsub
### Download publisher
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
### Download subscriber
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py

### Modify setup.py to add talker & listener to console_scripts
    entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
    },



## Build and run
cd ros2_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build --packages-select py_pubsub

source install/setup.bash


