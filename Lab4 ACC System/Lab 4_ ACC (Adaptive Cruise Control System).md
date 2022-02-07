# Lab 4: ACC (Adaptive Cruise Control System)

**TA**: Ginger (f03922104@ntu.edu.tw)


## Table of Contents


[toc]


## Changelog

- 2021-11-09 17:00 publication

## Overview

- Why ACC?
Vehicles will automatically adjusts the speed to maintain a safe distance from ahead.
ACC use lidar, camera, and etc to detect the distance between from the front car.


*Reference*: https://en.wikipedia.org/wiki/Adaptive_cruise_control


### Goal

In this lab, you will drive the car on a 300 meters straight lane, and there will be another NPC car in front of you. The NPC car will increase its speed between $5m/s^2$ and $10m/s^2$ in the first half of the lane, and decrease the speed between $-5m/s^2$ and $-10m/s^2$ in the last half of the lane. The speed of NPC car will between $5km/hr$ and $100km/hr$.

Your mission is to follow the NPC car with a properly distance based on your speed.

![](https://i.imgur.com/EOYgE2b.png)


## Prerequisites

Please follow the example setup before getting started.
[Example Setup for CSIE5317 Course (2021 Fall)](https://hackmd.io/@jerry73204/B1keEpewY)
The follows are required.

- Autoware.Auto docker container
- LGSVL simulator, version 2021.3


Download example code from [this link](https://cool.ntu.edu.tw/courses/8184/files/1307519). It contains the following files.

- `scenario.py`
- `controller.py`
- `sensor_config.json`


## Environment Setup

### Map Setup

- Make sure "SingleLaneRoad" map is installed.

### Vehicle Setup

Start LGSVL simlator and enter setup page in browser described in Lab 1.

1. Finish vehicle setup described in Lab 1. Select the car model "Lexus2016RXHybrid".
![](https://i.imgur.com/X9DygAs.png)

2. Enter the settings of "Lexus2016RXHybrid". Open the sensor configuration. Then, click "Add New Configuration". Name the configuration "Lab4 Sensors".
![](https://i.imgur.com/ALdlYh0.png)
![](https://i.imgur.com/UWdwziG.png)
![](https://i.imgur.com/pzPcqb8.png)

3. Fill the configuration as follows.
  (1). Open the JSON editor
  (2). Copy the content `sensor_config.json` to the editor
  (3). Click "Save"
  (4). Click "Exit"
![](https://i.imgur.com/0pDH0mU.png)

4. Copy the id of your sensor configuration.
Click the ID icon on the right side of sensor configuration, and paste it to a notebook. The id will be something like 'ad239c27-3259-48b1-be96-43a010849e1b.
![](https://i.imgur.com/EJL9xUZ.png)


### Simulation Setup

Create a API only simulation in the following steps. 

1. Click "Simulations" in the left menu. Click "Add New" button.
  ![](https://i.imgur.com/xTjk6PU.png)

2. Name the simluation "Lab4".
![](https://i.imgur.com/2DzpENN.png)

3. Choose "API Only" template.
![](https://i.imgur.com/tae8nS0.png)

4. Publish and run the Simulation.
![](https://i.imgur.com/GXaYtUR.png)

### Start the Autoware Container

Start the container installed in Lab1.

```shell=sh
cd ~/adehome/AutowareAuto
source .aderc-amd64-foxy-lgsvl-nvidia       
ade start -- -p 127.0.0.1:9090:9090 -p 127.0.0.1:8181:8181
```

In later instructions, we always run this command to enter the container.

```shell=sh
ade enter
source /opt/AutowareAuto/setup.bash
```

### Start LGSVL Bridge

Open a new terminal and enter the container. Run LGSVL bridge installed in Lab 1.

```shell=sh
# enter ade container, then
cd AutowareAuto
source install/setup.bash
lgsvl_bridge
```

### Control Your Car

1. Unzip `lab4.zip` file and move the unzipped `lab4` directory to `~/adehome/lab4`.
```
~/adehome
├── AutowareAuto
│   └── ...
├── lab4
│   ├── controller.py
│   ├── scenario.py
│   └── sensor_config.json
└── ...

```

2. Open a new terminal and enter the container. Run the vehicle controller program.

```shell=sh
# enter ade container, then
cd lab4
python3 controller.py
```

3. Add the vehicle setting, and run the Python API.

```shell=sh
# enter ade container, then
export LGSVL__VEHICLE_0=ad239c27-3259-48b1-be96-43a010849e1b
# replace the id with your owned id.
cd lab4 
python3 scenario.py
```

5. If your simulation setup is correct, the simulator will load the map and vehicle as the figure below.
![](https://i.imgur.com/Awk4vVc.png)

## Requirements

### Task Description

The task is to implement the AAC feature in the controller code `controller.py`. It runs in two stages: accelerate and then brake. The implementtion must satisfy the following requirements.

1. Follow the front car 1 to 5 seconds time-to-collision(TTC) distance.
2. The judge program will automatically calculate the average TTC.
3. You must not collide the front car.

### Write Your Controller

Same as Lab3.

[Write-Your-Controller](https://hackmd.io/@jerry73204/SJ3WIllIt#Write-Your-Controller)
The code template can be found in the `lab4.zip` file, or in the [Gist link](https://gist.github.com/jerry73204/2ad305ec79b5cda572870c00006516b7). Fill the code marked by `TODO` comments.


## Grading & Submission

Once the front car reach its destination, the judge program will be stopped. Your average TTC will be show on the terminal.

- 1 ~ 1.5 seconds: 90 points
- 1.5 ~ 2 seconds: 100 points
- 2 ~ 3 seconds: 80 points
- 3 ~ 4 seconds: 70 points
- 4 ~ 5 seconds: 60 points
- other: 0 points

Submit the following items to NTU COOL platform. Your score is granted ONLY when all items are met. 

- The controller program `controller.py`.
- The screenshot of simulation result.
    1. The final state of simulator.
    2. The judge messages from simulator terminal.

Example:
![](https://i.imgur.com/q3fBQ5m.png)