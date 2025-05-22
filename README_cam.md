# USB 카메라로부터 영상을 캡쳐하는 도커 생성 및 실행 방법
USB(/dev/video0) 장치로부터 영상을 캡쳐하여 /image_raw 토픽에 publish함

## Build usb-cam-foxy
```sh
docker build -t usb-cam-foxy -f Dockerfile_cam .
```

## Run usb-cam-foxy
```sh
docker run -it --rm --net host -e ROS_DOMAIN_ID=10  --device=/dev/video0:/dev/video0   --group-add video   usb-cam-foxy
```

