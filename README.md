# 自駕車模擬與訓練於各項任務
系統專題研究-物聯網中介軟體設計 | System Design Topic - Design for IoT Middleware | 施吉昇教授 | 2021 Fall

## Demo Video

## Setup Environment

### Prerequisites

```shell=sh
sudo apt update
sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    git
```


### Docker-ce Setup

```shell=sh
# Import GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install docker-ce
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io
```

Add your account to the `docker` group, and **reboot** to take effect.

```shell=sh
usermod -a -G docker $USER
# reboot your PC/laptop
```

### ROS 2 Setup

**Version**: ROS 2 foxy

```shell=sh
# Install GPG key
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

#  Add the repository to source list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-foxy-desktop
```

### LGSVL Simulator Setup

**Version**: 2021.3

Visit the official site (https://www.svlsimulator.com/) and download the version **2021.3**. The downloaded zip file name will be `svlsimulator-linux64-2021.3.zip`. Unzip the file to home directory.

```shell=sh
cd ~
unzip ~/Downloads/svlsimulator-linux64-2021.3.zip
```

### ADE Setup

**Version**: 4.3.0

Install the `ade` binary at `~/.local/bin`

```shell=sh
mkdir -p ~/.local/bin && cd ~/.local/bin
wget https://gitlab.com/ApexAI/ade-cli/-/jobs/1341322851/artifacts/raw/dist/ade+x86_64
mv ade+x86_64 ade
chmod +x ade
```

Add `~/.local/bin` to your PATH so that you can run `ade` command everywhere.

```shell=sh
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
exec $0  # restart bash
```



Check that the versoin is correct.

```shell=sh
ade --version
4.3.0
```

### Autoware.Auto Setup

**Version**: release-1.0.0

#### Download Autoware.Auto repository

```shell=sh
# Create workspace for ADE
mkdir -p ~/adehome &&cd ~/adehome
touch .adehome

# Download Autoware.Auto source code and check out to version 1.0.0
git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
cd AutowareAuto
git checkout tags/1.0.0 -b release-1.0.0
```

#### Enter ADE container (changed at 2021-11-07)

To enable GPU support, create the file `~/adehome/.aderc` with the content below. You may ignore this step if GPU support is not needed.

```shell=sh
export ADE_DOCKER_RUN_ARGS="--cap-add=SYS_PTRACE --net=host --privileged --add-host ade:127.0.0.1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --gpus all -ti --rm -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
-e NVIDIA_VISIBLE_DEVICES=all \
-e NVIDIA_DRIVER_CAPABILITIES=compute,utility,display"
export ADE_GITLAB=gitlab.com
export ADE_REGISTRY=registry.gitlab.com
export ADE_DISABLE_NVIDIA_DOCKER=false
export ADE_IMAGES="
  registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/amd64/ade-foxy:1.0.0
  registry.gitlab.com/autowarefoundation/autoware.auto/autowareauto/amd64/binary-foxy:1.0.0
  registry.gitlab.com/autowarefoundation/autoware.auto/ade-lgsvl/foxy:2021.3
  nvidia/cuda:11.0-base
"
```

Start the container.

```shell=sh
cd ~/adehome
ade start -- -p 127.0.0.1:9090:9090
```

If you encouter an error like this, you can either follow the instructions to force restarting or ignore the error.

~~~
ERROR: ade is already running. Use ade enter to enter or ade start -f to restart.
~~~


After the container is started, run this command to enter the continer environment. You can open multiple terminals and enter the container this way.

```shell=sh
ade enter
```

#### Download lgsvl_msgs and ros2-lgsvl-bridge

In this section, we're going to configure and build the Autoware.Auto within the container.

Prepare the folder.

```shell=sh
# enter ADE container
mkdir -p ~/AutowareAuto/src/external
cd ~/Autoware
vcs import < autoware.auto.$ROS_DISTRO.repos
```

Download lgsvl_msgs and check out to specified version. (**changed 2021-11-07**)

```shell=sh
cd ~/AutowareAuto/src/external
git clone https://github.com/lgsvl/lgsvl_msgs.git
cd lgsvl_msgs
git checkout 0.0.4
```


Download ros2-lgsvl-bridge and check out to specified version.

```shell=sh
cd ~/AutowareAuto/src/external
git clone https://github.com/lgsvl/ros2-lgsvl-bridge.git
cd ros2-lgsvl-bridge
git checkout 0.2.1
```

#### Build Autoware.Auto

```shell=sh
# inside the ADE container
cd ~/AutowareAuto
colcon build --cmake-args '-DCMAKE_BUILD_TYPE=Release'
```


### Start the Development Environment

There are two steps to go.
- Start the simulator on host system (without entering the ADE environment).
- Run the `lgsvl_bridge` within the ADE container.


```shell=sh
# On host system, OUTSIDE pf the ADE container
source ~/svlsimulator-linux64-2021.3
./simulator
```

```shell=sh
# inside the ADE container
source ~/AutowareAuto/install/setup.bash
lgsvl_bridge
```

## Getting Started on Each Task
Please see each Lab* directory for more details.

