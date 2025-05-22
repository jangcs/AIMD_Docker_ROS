# 이미지를 2단계 방식으로 수신하는 도커 생성 및 실행 방법
토픽(/aimd/imgs)을 통해 이미지에 대한 저장 위치를 수신하고, 해당 저장 위치의 이미지를 읽어서 화면에 보여줌

## Build img-path-sub
```sh
docker build -t img_path_sub -f Dockerfile_img_path_sub .
```

## Run img-path-sub
### Xhost in the host
```sh
xhost +local:docker
```
### run docker
```sh
docker run -it --rm --net host --ipc host --name img_view -e DISPLAY=unix$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume ./data:/data img_path_sub
```

