## Build usb-cam-foxy
```sh
docker build -t usb-cam-foxy -f Dockerfile_cam .
```

## Run usb-cam-foxy
```sh
docker run -it --rm   --device=/dev/video0:/dev/video0   --group-add video   usb-cam-foxy
```

