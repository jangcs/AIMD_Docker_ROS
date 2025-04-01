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
docker run --privileged -it --rm --net host --ipc host --name img_view -e DISPLAY=unix$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume ./data:/data img_path_sub
```

