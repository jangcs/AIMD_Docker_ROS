# Docker 설치
## Docker CE(Community Edition) 설치를 추천
```sh
apt-get update
sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```
## Docker compose plugin 설치
```sh
sudo apt-get install docker-compose-plugin
```

## Add user to docker group
```sh
sudo groupadd -f docker
sudo usermod -aG docker $USER
```

## Nvidia Container Toolkit 설정
### Stable repository 및 GPG key 설정
```sh
distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```
### install nvidia-docker
```sh
sudo apt-get update
sudo apt-get install -y nvidia-docker2
```

### restart docker daemon
```sh
sudo systemctl restart docker
```

### base CUDA container 테스트
```sh
sudo nvidia-docker run --rm --gpus all nvidia/cuda:12.2.2-base-ubuntu20.04 nvidia-smi
```

# Build Docker Images
## Build a publisher 
```sh
docker build --tag pub:aimd -f ./Dockerfile_pub .
```

## Build a subscriber
```sh
docker build --tag sub:aimd -f ./Dockerfile_sub .
```
# RUN 
## Run a publisher 
```sh
docker run -it --rm --name publisher --network="host" pub:aimd
```
## or run a publisher with GPU
```sh
nvidia-docker run -it --rm --gpus all --name publisher --network="host" pub:aimd
```

## Run a subscriber
```sh
docker run -it --rm --name subscriber --network="host" sub:aimd
```
## or run a subscriber with GPU
```sh
nvidia-docker run -it --rm --gpus all --name subscriber --network="host" sub:aimd
```

# Run with docker compose

```sh
cat docker-compose.yml
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

```sh
docker compose up
```

다른 docker compose 파일을 이용하기 위해서는
```sh
docker compose -f <docker-compose-file.yml> up
```

# Run with K8s
minikube를 사용한다고 가정하고 진행함.
## Upload container images to minikube
```sh
minikube image load pub:aimd
minikube image load sub:aimd
```
## Deploy a pod sample(talker/listener) and log it(listener) 
```sh
source k8s_ros.sh
```
or
```sh
kubectl apply -f k8s_pubsub.yaml
kubectl logs --follow `kubectl get pods | grep ros-pubsub | head -n 1 | awk '{print $1}'` -c ros-sub
```
## Local에서 ROS2로 Pod 내부의 Container와 통신
ROS2 Node를 실행하기 위해서 아래 환경 설정이 필요함 
```sh
export CYCLONEDDS_URI=`pwd`/cyclonedds.xml
```
설정을 완료한 후, Local host에서 ROS2 Node를 실행하여 통신을 진행

# Local Host에 ROS Setup 과정 정리 (py_pubsub sample 생성 과정) 
local에 ros2를 설치할 때 만 참고하면 됨. <br>
Docker만 사용할 경우 ros2_ws/src/py_pubsub에 이미 만들어 놨으므로 아래 내용을 추가로 실행할 필요없음.
## Create a ROS workspace
```sh
mkdir -p ~/ros2_ws/src
```

## Create a package (py_pubsub sample)
```sh
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub
```

## Download sample pub/sub codes & update setup.py

### Download publisher code
```sh
cd ~/ros2_ws/src/py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
### Download subscriber code
```sh
cd ~/ros2_ws/src/py_pubsub/py_pubsub
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

### Modify setup.py (add talker & listener to console_scripts)
```sh
vi ~/ros2_ws/src/py_pubsub/setup.py
```
    entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
    },


## Build the package
```sh
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build 
```

## Source setup.bash (local overlay setup)
```sh
cd ~/ros2_ws
source install/setup.bash
```

## Run pub(talker)/sub(listener) 
```sh
ros2 run py_pubsub talker
ros2 run py_pubsub listener
```

# 참고
## colcon build 중 에러가 발생하는 경우
```sh
pip uninstall empy
pip install empy==3.3.4

pip install catkin_pkg
pip install numpy
pip install lark
```
