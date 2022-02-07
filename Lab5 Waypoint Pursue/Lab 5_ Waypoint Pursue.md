# Lab 5: Waypoint Pursue

**TA**: Jerry (d10922013@ntu.edu.tw)

## Table of Contents

[toc]


## Changelog

- 2021-11-30 15:00 Homework published
- 2021-12-14 11:00 Fix erroneous image

## Overview

In this homework, implement a vehicle controller to read waypoint positions from a CSV file in the beginning, and controls the vehicle to pursue the waypoints one by one. The score is calculated according to the elapsed time and minimum distances to each waypoint.

The image illustrates the path on the Shalun map. There are 20 waypoints sampled on the path. The example execution can be found in the [video](https://cool.ntu.edu.tw/files/1413094/download?download_frd=1).

![](https://i.imgur.com/NdvkCJo.jpg)



## Prerequisites

- Please follow the example setup before getting started.
[Example Setup for CSIE5317 Course (2021 Fall)](https://hackmd.io/@jerry73204/B1keEpewY)

- Python API is installed in ADE container. You may check [Lab 3](https://hackmd.io/@jerry73204/SJ3WIllIt) for detailed steps.

- Download the materials from [this link](https://cool.ntu.edu.tw/courses/8184/files/1420536/download?download_frd=1). It contains the following files.
  - `scenario.py`
  - `controller.py`
  - `sensor_config.json`
  - `waypoints.csv`

## Environment Setup

### Map Setup

- Make sure "Shalun" map is installed.
![](https://i.imgur.com/Juciwx6.png)


### Vehicle Setup

Start LGSVL simlator and enter setup page in browser described in Lab 1.

1. Finish vehicle setup described in Lab 1. Select the car model "Lexus2016RXHybrid".
![](https://i.imgur.com/X9DygAs.png)

2. Enter the settings of "Lexus2016RXHybrid". Open the sensor configuration. Then, click "Add New Configuration". Name the configuration "Lab5 Sensors".

![](https://i.imgur.com/ALdlYh0.png)

![](https://i.imgur.com/UWdwziG.png)

![](https://i.imgur.com/KJcfkRR.png)

3. Update the configuration as follows.
  (1). Click the upload button.
  (2). Select `sensor_config.json`
  (3). Click "Save" and then "Exit"

![](https://i.imgur.com/omlLKgW.png)

![](https://i.imgur.com/ks7vtft.png)

4. Check if "Lab5 Sensors" shows up in your list. Click the "ID" button along side the entry to save the UUID.

![](https://i.imgur.com/tKh1nkD.png)

![](https://i.imgur.com/Tgs5w6C.png)


### Simulation Setup

Create a API only simulation in the following steps. 

1. Click "Simulations" in the left menu. Click "Add New" button.
  ![](https://i.imgur.com/xTjk6PU.png)

2. Name the simluation "Lab5".
![](https://i.imgur.com/lTE4oG3.png)


3. Choose "API Only" template.
![](https://i.imgur.com/soQnVkP.png)

5. Click "Next" in remaining steps. The simulation profile will like this.
![](https://i.imgur.com/pG9if0A.png)

### Start the Autoware Container

Start the container installed in Lab1.

```shell=sh
cd ~/adehome/AutowareAuto
ade start -- -p 127.0.0.1:9090:9090 -p 127.0.0.1:8181:8181
```

In later instructions, we always run this command to enter the container.

```shell=sh
ade enter
source AutowareAuto/install/setup.sh
```

## Run Waypoint Pursuing Simulation

### Prepare the files

1. Unzip `lab5.zip` file and move the unzipped `lab5` directory to `~/adehome/lab5`. The diretory structure looks like this.
```
~/adehome
├── AutowareAuto
│   └── ...
├── lab5
│   ├── controller.py
│   ├── scenario.py
│   ├── waypoints.csv
│   └── sensor_config.json
└── ...

```

### Start LGSVL Bridge

Open a new terminal and enter the container. Run LGSVL bridge installed in Lab 1.

```shell=sh
# enter ade container, then
lgsvl_bridge
```

### Run Simulator

Start the LGSVL simulator. It's not needed to enter the ADE container beforehand.

```shell=sh
cd ~/svlsimulator-linux64-2021.3  # this path may differ from your setup
./simulator
```

Once the simulator is started, click "Run Simulation" in the brower.

![](https://i.imgur.com/Z1IiZ91.png)

The simulator window will show "API ready!" on the screen.

![](https://i.imgur.com/we1tCw9.png)


### Run `scenario.py`

Start another terminal and enter the container. Set the `LGSVL__VEHICLE_0` envorinment variable to the UUID copied during the vehicle setup, and run `scenario.py`.

The program tracks passed waypoints listed in `waypoints.csv` and computes score accordingly.

```shell=sh
cd lab5
export LGSVL__VEHICLE_0=fb0029ae-a9f7-467f-83e7-3c1cc2c69f6a
python3 scenario.py
```

### Run `controller.py`

Open a new terminal and enter the container. Run `controller.py` to control the vehicle. **You need to modify the program to finish the homework.**

```shell=sh
# enter ade container, then
cd lab5
python controller.py
```

## Requirements

### Task Description

Implement the `controller.py` to finish the following tasks.

1. Load waypoint positions from `waypoints.csv` in the same directory.
2. Control the car such that it reaches the waypoints, one by one in the order listed in `waypoints.csv`.
3. Stop the car after the last waypoint is visited.

During the driving, it's NOT required to respect the traffic signal.

### Implementation Details


The `Driver` class in the code is the main controller. It has several callback methods.

```python
class Driver(Node):
    def __init__(self):
        # omit..
        pass
        
    def position_callback(self):
        # omit..
        pass
        
    def controller_callback(self):
        # omit..
        pass
```

The `position_callback()` is called whenever a new vehicle position is available. It can used to measure the distance to the recent waypoint position.

```python
    def position_callback(self, data):
        # TODO: store vehicle position
        print("Current pos: %f, %f, %f" % (data.x, data.y, data.z))
```

The `controller_callback()` is called periodically. The method sends controlling commands to the vehicle.

```python
    def controller_callback(self):
        # TODO: implement your car controller

        msg = VehicleControlData()
        msg.acceleration_pct = 0.0
        msg.braking_pct = 0.0

        self.pub.publish(msg)
```

## Grading

### Grading Items

The following factors are considered for grading.

#### Distance to each waypoint (80%)

The `scenario.py` keeps track of minimum distances to every waypoint. The scoring of each waypoint falls into the following categories. The final score is averaged over all waypoint scores.

Waypoint score (distance measured in meters):

- `<=0.5`: 100 points
- `>0.5, <=2.0`: (2 - distance) * 20 / 1.5 + 80 points
- `>2.0, <= 3.0`: (3.0 - distance) * 20.0 + 60.0 points
- `>3.0`: 0 points

#### Elapsed time (20%)

The `scenario.py` counts the elapsed time since the vehicle starts moving until the vehicle stops after passing the last waypoint. The scoring falls in the following cases.

- `<=75`: 100 points
- `>75, <=100`: 80 points
- `>100, <=150`: 60 points
- `>150`: 0 points

### Scoring

If the task is finished in 150 seoncds limit, the final score is computed by the formula.

```
final_score = average_distance_score * 0.8 + elapsed_time_score * 0.2
```

The scoring details can be found in the terminal output of `scenario.py`.

![](https://i.imgur.com/mH7CJZs.png)

## Submission

Please upload the following files for submission.

- `controller.py`
- The screenshot of terminal output and simulator window named in `STUDENT_ID.png`, in well-known image format. For example, `D10922013.png`, `ntust_xxxxx.png`, `R06922113.jpg`.

![](https://i.imgur.com/HUI3rnV.png)
