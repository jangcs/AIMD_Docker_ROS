# 이미지를 2단계 방식으로 전송하는 도커 생성 및 실행 방법
/image_raw에 subscribe하여 이미지를 얻어와서, /data 디렉토리에 저장한 후, 저장 위치를 json 메시지로 토픽(/aimd/imgs)에 publish 함 

## Build img_path_pub
```sh
docker build -t img_path_pub -f Dockerfile_img_path_pub .
```

## Run img_path_pub
```sh
docker run -it --rm --net host --name img_path_pub --volume ./data:/data img_path_pub
```

