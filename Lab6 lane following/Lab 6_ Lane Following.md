# Lab 6: Lane Following

**TA**: Ginger (f03922104@ntu.edu.tw)


## Table of Contents


[toc]


## Changelog

- 2021-12-14 17:00 publication

### Goal

The route will consist a straight lane and a curve.
At the T-junction, you should just go forward.
No need to stop. The judging program will stop automatically after you pass the last checkpoint.


In this lab, you will drive the car on a route consists straight lanes and curves. You need to go forward at T-junctions.
Your mission is to keep the ego car in the middle of the lane by the information from the camera.

![](https://i.imgur.com/MsBPDpp.png)

You need to drive at the inner lane of the <font color="#f00">red route</font> on the image.


## Prerequisites

Please follow the example setup before getting started.
[Example Setup for CSIE5317 Course (2021 Fall)](https://hackmd.io/@jerry73204/B1keEpewY)
The follows are required.

- Autoware.Auto docker container
- LGSVL simulator, version 2021.3


Download example code from [this link](https://cool.ntu.edu.tw/courses/8184/files/1482191). It contains the following files.

- `scenario.py`
- `controller.py`
- `sensor_config.json`
- `show_image.py`


## Environment Setup


### Vehicle Setup

Start LGSVL simlator and enter setup page in browser described in Lab 1.

1. Finish vehicle setup described in Lab 1. Select the car model "Lexus2016RXHybrid".

2. Enter the settings of "Lexus2016RXHybrid". Open the sensor configuration. Then, click "Add New Configuration". Name the configuration "Lab6 Sensors".
![](https://i.imgur.com/ALdlYh0.png)
![](https://i.imgur.com/UWdwziG.png)

3. Fill the configuration as follows.
  (1). Open the JSON editor
  (2). Copy the content `sensor_config.json` to the editor
  (3). Click "Save"
  (4). Click "Exit"
![](https://i.imgur.com/0pDH0mU.png)

4. Copy the id of your sensor configuration.
Click the ID icon on the right side of sensor configuration, and paste it to a notebook. The id will be something like 'ad239c27-3259-48b1-be96-43a010849e1b.


### Simulation Setup

Create a API only simulation in the following steps. 

1. Click "Simulations" in the left menu. Click "Add New" button.
  ![](https://i.imgur.com/xTjk6PU.png)

2. Name the simluation "Lab6".
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

### Install OpenCV Python Package
```shell=sh
# enter ade container, then
pip3 install opencv-python
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

1. Unzip `lab6.zip` file and move the unzipped `lab6` directory to `~/adehome/lab6`.
```
~/adehome
├── AutowareAuto
│   └── ...
├── lab6
│   ├── controller.py
│   ├── scenario.py
│   ├── sensor_config.json
│   └── show_image.py
└── ...

```

2. Open a new terminal and enter the container. Run the vehicle controller program.

```shell=sh
# enter ade container, then
cd lab6
python3 controller.py
```

3. Add the vehicle setting, and run the Python API.

```shell=sh
# enter ade container, then
export LGSVL__VEHICLE_0=ad239c27-3259-48b1-be96-43a010849e1b
# replace the id with your owned id.
cd lab6
python3 scenario.py
```

5. If your simulation setup is correct, the simulator will load the map and vehicle as the figure below.
![](https://i.imgur.com/B8T01bg.png)


### Test Camera Image Receiving

You can receive the camera image from `/simulator/camera_node/image/compressed` topic, and show it on window by the following code.

```shell=sh
# enter ade container, then
cd lab6
python3 show_image.py
```

```python
# In show_image.py
def camera_callback(self, data):
    image_format = data.format
    image_data = np.array(data.data, dtype=np.uint8)
    image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
    print('image format: ', image_format,', size: ',image.shape)
    cv2.imwrite('tmp.jpg', image)
    cv2.imshow('windows', image)
    cv2.waitKey(1)
```
It will pop up a window and show the image it receive from the camera on car.

![](https://i.imgur.com/CDYdhd2.png)

## Requirements

### Task Description

The task is to implement the lane following feature in the controller code `controller.py`.
There are two curves and three T-junctions.

1. Drive your car in the middle of inner lane.
2. The elapsed time should be less than 90 seconds.

### Write Your Controller

Same as Lab3.

The code template can be found in the `lab6.zip` file. You can implement the controller by timer or camera callback.


## Grading & Submission

The `scenario.py` keeps track of minimum distances to every waypoint. The scoring of each waypoint falls into the following categories. The final score is averaged over all waypoint scores.

Waypoint score (distance measured in meters):

- `<=0.3`: 100 points
- `>0.3, <=1.3`: 100 - (distance - 0.3) * 100 points
- `>1.3`: 0 points

Submit the following items to NTU COOL platform. Your score is granted ONLY when all items are met. 

- The controller program `controller.py`.
- The screenshot of simulation result.
    1. The final state of simulator.
    2. The judge messages from simulator terminal.
        a. Distance to all waypoints
        b. Final score
        c. Time used
        
Example:
![](https://i.imgur.com/nqRpD0V.png)