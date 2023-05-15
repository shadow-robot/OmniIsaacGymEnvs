xhost +
docker create --name isaac-sim-oige --entrypoint bash -it --gpus all -e "ACCEPT_EULA=Y" --network=host \
-v $HOME/.Xauthority:/root/.Xauthority \
-e DISPLAY \
-v /etc/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json \
-v /etc/vulkan/implicit_layer.d/nvidia_layers.json:/etc/vulkan/implicit_layer.d/nvidia_layers.json \
-v /usr/share/glvnd/egl_vendor.d/10_nvidia.json:/usr/share/glvnd/egl_vendor.d/10_nvidia.json \
-v ${PWD}:/workspace/omniisaacgymenvs \
-v /home/tom/isaac_usd_assets:/workspace/isaac_usd_assets \
-v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
-v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
-v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
-v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
-v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
-v ~/docker/isaac-sim/config:/root/.nvidia-omniverse/config:rw \
-v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
-v ~/docker/isaac-sim/documents:/root/Documents:rw \
-v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache/Kit:rw \
nvcr.io/nvidia/isaac-sim:2022.2.1

docker start isaac-sim-oige

if [[ $(hostname) == "Athena" ]]; then
  echo "On Athena, enabling apt cache proxy..."
  docker exec -it isaac-sim-oige sh -c "echo 'Acquire::http::Proxy \"http://localhost:3142\";' >> /etc/apt/apt.conf.d/00aptproxy"
fi

docker exec -it isaac-sim-oige sh -c 'echo "export USE_MUJOCO=True" >> ~/.bashrc'
docker exec -it isaac-sim-oige sh -c "apt update && apt install debconf-utils && debconf-set-selections <<< 'keyboard-configuration keyboard-configuration/variant select English (UK)' && debconf-set-selections <<< 'keyboard-configuration keyboard-configuration/layoutcode string English (UK)'"
docker exec -it isaac-sim-oige sh -c "apt update && DEBIAN_FRONTEND=noninteractive TZ=Europe/London apt-get install -y tzdata && DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration && dpkg-reconfigure keyboard-configuration -f noninteractive"
docker exec -it isaac-sim-oige sh -c "cd /workspace/omniisaacgymenvs && /isaac-sim/python.sh -m pip install -e . && cd omniisaacgymenvs"
docker exec -it isaac-sim-oige sh -c "apt install -y nano curl wget git highlight gnome-terminal"
docker exec -it isaac-sim-oige sh -c "wget -O ~/.bash_functions https://raw.githubusercontent.com/carebare47/useful_things/master/bash_functions"
docker exec -it isaac-sim-oige sh -c "echo \"source ~/.bash_functions\" >> ~/.bashrc"
docker exec -it isaac-sim-oige sh -c "git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf && ~/.fzf/install --all"
# docker exec -it isaac-sim-oige sh -c "bash -c <(curl -Ls bit.ly/tom_setup | grep highlight | grep cat | sed -r 's/sudo//g')"

docker exec -it -w /workspace/omniisaacgymenvs/omniisaacgymenvs isaac-sim-oige bash



