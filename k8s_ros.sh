export CYCLONEDDS_URI=`pwd`/cyclonedds.xml
kubectl apply -f k8s_pubsub.yaml
pod=`kubectl get pods | grep ros-pubsub | head -n 1 | awk '{print $1}'`
kubectl logs --follow $pod -c ros-sub
