## Build img_view
```sh
docker build -t img_view -f Dockerfile_imgview .
```


## Run img_view
### Xhost in the host
```sh
xhost +local:docker
```
### run docker
```sh
docker run --privileged -it --rm --net host --ipc host --name img_view -e DISPLAY=unix$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix:ro --volume ./data:/data img_view
```

