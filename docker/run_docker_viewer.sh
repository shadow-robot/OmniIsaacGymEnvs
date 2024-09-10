xhost +
CONTAINER_NAME="isaac-sim-oige-2023"
docker run --name $CONTAINER_NAME --entrypoint bash -it -d --gpus all -e "ACCEPT_EULA=Y" --rm --network=host \
-v $HOME/.Xauthority:/root/.Xauthority \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
-v ${PWD}:/workspace/omniisaacgymenvs \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
nvcr.io/nvidia/isaac-sim:2023.1.0-hotfix.1

# docker exec -it $CONTAINER_NAME sh -c "echo 'Acquire::http::Proxy \"http://localhost:3142\";' >> /etc/apt/apt.conf.d/00aptproxy"


docker exec -it $CONTAINER_NAME sh -c "cd /workspace/omniisaacgymenvs && /isaac-sim/python.sh -m pip install -e . && cd omniisaacgymenvs"
docker exec -it -w /workspace/omniisaacgymenvs/omniisaacgymenvs $CONTAINER_NAME bash
