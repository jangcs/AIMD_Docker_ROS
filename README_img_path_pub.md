## Build img_path_pub
```sh
docker build -t img_path_pub -f Dockerfile_img_path_pub .
```

## Run img_path_pub
```sh
docker run -it --rm --net host --name img_path_pub --volume ./data:/data img_path_pub
```

