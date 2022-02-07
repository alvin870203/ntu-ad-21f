# Lab 3: AEBS (Advanced Emergency Braking System)

**TA**: Jerry (d10922013@ntu.edu.tw)


## 2021-11-04 Update

- Many students report that the setup from Lab 1/2 does not work with Lab 3. Here we provide the example setup procedure that works with modern Autoware and simulator. Please report to TA if you still encounter issues.

   https://hackmd.io/@jerry73204/B1keEpewY

- The sensor configuration file `sensor_config.json` gives insufficient distance to detect neighboring car. The issue is fixed and please update the file from this link.

  https://cool.ntu.edu.tw/courses/8184/files/1290418/download?download_frd=1

- Since many students have hard time on environment setup. The deadline is postponed by one week to Nov. 16.

## Table of Contents


[toc]


## Changelog

- 2021-10-26 15:00 publication
- 2021-10-26 16:26 minor edit
- 2021-11-04 15:34 update judge script, announce example environment setup

## Overview

In 2017, there are 30% of traffic accident which cause death is because drivers don’t notice the front car when driving. The AEBS system is designed to prevent this situation. It works with a distance sensor, either a radar or a LiDAR, to detect if there is a car at the front, and takes control of the car if needed.

### Regulations

- Tested vehicle should speed up to 80km/h and have 120m distance from the front car.
- When approaching the front car, there should be some alarms.
  - TTC is from 3 to 4.4 second.
- When the distant between front car is too close, the braking system start working.
  - TTC last than 3 second.
- If the front car is not moving, the vehicle should slow down more than 10km/h or 20km/h. If the front car is moving, the collision should not happen.

![](https://i.imgur.com/tPc2TbL.png)


*Reference*: http://vsccdms.vscc.org.tw/webfile/Epaper/500000225/File/fbb72b7e-1534-43ad-a73c-eaa23c6c139d.pdf

### Goal

In this homework, the controlled car starts before a stop line, and a static car is roughly 50 meters away. Write a controller to drive your car. The controlled car starts to brake when it sees the front car. It must safely stop without collision. The positoin of front car is ramdomly initialized.

![](https://i.imgur.com/yPZAqXs.png)


## Prerequisites

Please finish Lab 1 & Lab2 before getting started. The follows are required.

- Autoware.Auto docker container
- LGSVL simulator, version 2021.3


Download example code from [this link](https://cool.ntu.edu.tw/courses/8184/files/1290418/download?download_frd=1) **(changed on Nov 4)**. It contains the following files.

- `scenario.py`
- `controller.py`
- `sensor_config.json`

## Environment Setup

### Map Setup

- Make sure "Shalun" map is installed.
![](https://i.imgur.com/Juciwx6.png)


### Vehicle Setup

Start LGSVL simlator and enter setup page in browser described in Lab 1.

1. Finish vehicle setup described in Lab 1. Select the car model "Lexus2016RXHybrid".
![](https://i.imgur.com/X9DygAs.png)

2. Enter the settings of "Lexus2016RXHybrid". Open the sensor configuration. Then, click "Add New Configuration". Name the configuration "Lab3 Sensors".
![](https://i.imgur.com/ALdlYh0.png)
![](https://i.imgur.com/UWdwziG.png)
![](https://i.imgur.com/ixFAQoG.png)

3. Fill the configuration as follows.
  (1). Open the JSON editor
  (2). Copy the content `sensor_config.json` to the editor
  (3). Click "Save"
  (4). Click "Exit"
![](https://i.imgur.com/0pDH0mU.png)

4. Check if "Lab3 Sensors" shows up in your list.
#![](https://i.imgur.com/wHuqz33.png)

### Simulation Setup

Create a API only simulation in the following steps. 

1. Click "Simulations" in the left menu. Click "Add New" button.
  ![](https://i.imgur.com/xTjk6PU.png)

2. Name the simluation "Lab3".
![](https://i.imgur.com/kN62kDY.png)

3. Choose "Python API" template, "Shalun" map and "Lexus2016RXHybrid" vehicle with "Lab3 Sensors" sensor configurtion. Copy the content of `scenario.py` into the Python Script editor.
![](https://i.imgur.com/uCzcv7S.png)

4. Select "Autoware.Auto" autopilot.
![](https://i.imgur.com/MoX1z7Z.png)

5. Check if your simulation setup is correct.
![](https://i.imgur.com/eydPoMJ.png)

### Start the Autoware Container

Start the container installed in Lab1.

```shell=sh
cd ~/adehome/AutowareAuto
source .aderc-amd64-foxy-lgsvl-nvidia       
ade start -- -p 127.0.0.1:9090:9090
```

In later instructions, we always run this command to enter the container.

```shell=sh
ade enter
source /opt/AutowareAuto/setup.bash
```


### Install Python API

This command installs API version 2021.3. Enter the container before proceeding.

```shell=sh
# enter ade container, then
pip3 install --upgrade --user git+https://github.com/lgsvl/PythonAPI.git@2021.3
```

### Start LGSVL Bridge

Open a new terminal and enter the container. Run LGSVL bridge installed in Lab 1.

```shell=sh
# enter ade container, then
cd AutowareAuto
source install/setup.bash
lgsvl_bridge
```

The example setup is shown the figure below.

![](https://i.imgur.com/DuaQA7f.png)

### Control Your Car

1. Unzip `lab3.zip` file and move the unzipped `lab3` directory to `~/adehome/lab3`.
```
~/adehome
├── AutowareAuto
│   └── ...
├── lab3
│   ├── controller.py
│   ├── scenario.py
│   └── sensor_config.json
└── ...

```

2. Open a new terminal and enter the container. Run the vehicle controller program.

```shell=sh
# enter ade container, then
cd lab3
python controller.py
```

The figure below shows the example execution.

![](https://i.imgur.com/IcVenW6.png)

3. Run the LGSVL simulator. Note that there is no need to enter the container.

```shell=sh
cd ~/svlsimulator-linux64-2021.3  # this path may differ from your setup
./simulator
```

4. In the simulator browser page, run the "Lab3" simulation configured in previous section.
![](https://i.imgur.com/2PJIdaY.png)

5. If your simulation setup is correct, you will see a car in front of the controlled vehicle.
![](https://i.imgur.com/O0TqvPX.png)

## Requirements

### Task Description

The task is to implement the AEBS feature in the controller code `controller.py`. It runs in two stages: accelerate and then brake. The implementtion must satisfy the following requirements.

1. Accelerate the car to 40km/h at minimum.
2. Brake and stop behind the front car by 0.5 to 10 meters without collision.

### Write Your Controller

The code template can be found in the `lab3.zip` file, or in the [Gist link](https://gist.github.com/jerry73204/2ad305ec79b5cda572870c00006516b7). Fill the code marked by `TODO` comments.

The `Driver` class in the code is the main controller. It has three callback methods.

```python
class Driver(Node):
    def __init__(self):
        # omit..
        pass
        
    def bounding_boxes_callback(self):
        # omit..
        pass
        
    def odom_callback(self):
        # omit..
        pass
        
    def controller_callback(self):
        # omit..
        pass
```

The `bounding_boxes_callback()` is called when object detection data from sensor is available. It is used to locate the front car.

```python
    def bounding_boxes_callback(self, data):
        # TODO: scan neighboring cars
        print('There are %d objects.' % len(data.detections))

        for index, det in enumerate(data.detections):
            box = det.bbox
            print('object %d: central of box is at %f %f %f.' %
                  (index, box.position.position.x, box.position.position.y,
                   box.position.position.z))
```

The `odom_callback()` is called when message from odometry sensor is available.

```python
    def odom_callback(self, data):
        # TODO: store vehicle position
        position = data.pose.pose.position
        print('Current pos: %f, %f, %f' % (position.x, position.y, position.z))
```

The `controller_callback()` is called periodically. The method sends controlling commands to the vehicle.`

```python
    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        msg.acceleration_pct = 0.0
        msg.braking_pct = 1.0

        self.pub.publish(msg)
```

## Grading & Submission

The grading depends on the distance from your car to the front car once your car stops. The car is considered stopped if the speed is not greater than 5km/h.

- 0.5 ~ 1.5 meters: 90 points
- 1.5 ~ 2.5 meters: 100 points
- 2.5 ~ 6 meters: 80 points
- 6 ~ 10 meters: 70 points
- other: 0 points

Submit the following items to NTU COOL platform. Your score is granted ONLY when all items are met. 

- The controller program `controller.py`.
- The screencast of simulation execution. (See example video [here](https://drive.google.com/file/d/1vaiwXgu44sEy1oys43JNydSCTrdxSbBu/view?usp=sharing)) See Recording Tutorial to learn instructions.
    1. Start by clicking `Run Simulation` button in browser.
    2. The recording from acceleration to braking.
    3. The judge message from simulator terminal.

### Recording Tutorial

- On Ubuntu, press Ctrl+Shift+Alt+R to start recording a screencast. Press again to stop the recording. Browse to `~/Videos` directory to find the video file.
- After the simulation finishes, press ESC on simulator to show the judge messages.