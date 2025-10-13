```bash
* Clone this repo
git@github.com:MSK-KWU/GTSAM-tutorial.git


* Docker build
docker build -f Dockerfile.gtsam -t gtsam-tutorial .


* Docker run with bashfile
bash docker_run.bash

* make build directory and build, make 

mkdir build && cd bulid
cmake ..
make 

./gtsam_factor_graph


