xhost +local:docker

docker run -it --rm \
  -v ~/codes/GTSAM-tutorial:/gtsam \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  gtsam-tutorial
