import random
import os
from environs import Env
import rclpy
from rclpy.node import Node
from lgsvl_msgs.msg import Detection3DArray, Detection3D

import lgsvl

env = Env()

LGSVL__SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
LGSVL__SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
LGSVL__AUTOPILOT_0_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
LGSVL__AUTOPILOT_0_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)
LGSVL__MAP = env.str("LGSVL__MAP", "Shalun")
LGSVL__VEHICLE_0 = env.str("LGSVL__VEHICLE_0", None)

if not LGSVL__VEHICLE_0:
    print('Please follow and instractions on document to retrieve the id of the vehicle.')
    exit(1)

EGO_POS = (-25.8, 0.0, 32.6)
EGO_ROT_Y = 249


INIT_X = 87.05
INIT_Y = 0
INIT_Z = 28.13

CAR_ANGLE = 337.9
EGO_POS = (INIT_X, INIT_Y, INIT_Z)
EGO_ROT_Y = CAR_ANGLE

DIS_THROSHOLD = 3
WAY_POINTS = [
(82.81, 38.59),
(77.51, 49.29),
(68.77, 57.71),
(53.97, 61.33),
(33.68, 54.92),
(14.93, 47.80),
(-6.05, 39.95),
(-23.28, 33.64),
(-49.65, 23.98),
(-60.75, 19.48),
(-67.79, 14.15),
(-69.64, 8.17),
(-68.37, 2.35)
]

distance2points = [99 for _ in range(len(WAY_POINTS))]

def main():
    rclpy.init()
    # Initialize or reset simulation.
    sim = lgsvl.Simulator(address=LGSVL__SIMULATOR_HOST,
                          port=LGSVL__SIMULATOR_PORT)
    if sim.current_scene == LGSVL__MAP:
        sim.reset()
    else:
        sim.load(scene=LGSVL__MAP, seed=650387)

    # Setup ego vehicle
    state = lgsvl.AgentState()
    state.transform.position = lgsvl.Vector(*EGO_POS)
    state.transform.rotation.y = EGO_ROT_Y
    ego = sim.add_agent(name=LGSVL__VEHICLE_0,
                        agent_type=lgsvl.AgentType.EGO,
                        state=state)
    ego.connect_bridge(LGSVL__AUTOPILOT_0_HOST, LGSVL__AUTOPILOT_0_PORT)

    finished = False
    obstacle_count = 0
    def on_collision(agent1, agent2, contact):
        nonlocal obstacle_count
        nonlocal sim, finished

        if agent1 is not None and agent2 is not None:
            print("Collision occurred. Test failed")
            if contact is not None:
                print("{} collided with {} at {}".format(
                    agent1.name, agent2.name, contact))
            else:
                print("{} collided with {}".format(agent1.name, agent2.name))
            finished = True

        elif obstacle_count >= 2:
            print("Collision occurred. Test failed")
            if contact is not None:
                print("The car hits an obstacle at {}".format(contact))
            else:
                print("The car hits an obstacle")
            sim.stop()

        else:
            obstacle_count += 1
            
    

    ego.on_collision(on_collision)
    # Starts simulation.
    last = -1
    timeUsed = 0
    point_idx = 0
    print('judge started')
    while not finished:
        sim.run(0.1)
        # print(ego.state.position)
        timeUsed += 0.1
        if last > 0:
            last -= 1
            if last <= 0:
                finished = True
        # Update distance
        distance = ((ego.state.position.x - WAY_POINTS[point_idx][0])**2 + (ego.state.position.z - WAY_POINTS[point_idx][1])**2)**0.5
        if point_idx == len(WAY_POINTS)-1 and distance2points[point_idx] < distance and last < 0:
            last = 10
        if distance2points[point_idx] > distance and distance < DIS_THROSHOLD:
            if distance2points[point_idx] == 99:
                print('Check point %d passed.'%(point_idx))
                if point_idx == 0:
                    timeUsed = 0
            distance2points[point_idx] = distance
        if point_idx < len(WAY_POINTS) - 1:
            distance2 = ((ego.state.position.x - WAY_POINTS[point_idx + 1][0])**2 + (ego.state.position.z - WAY_POINTS[point_idx + 1][1])**2)**0.5
            if distance2points[point_idx + 1] > distance2 and distance2 < DIS_THROSHOLD and distance2 < distance:
                point_idx += 1
                if distance2points[point_idx] == 99:
                    print('Check point %d passed.'%(point_idx))
                distance2points[point_idx] = distance2
    print('distances:', distance2points)
    score = 0.
    for distance in distance2points:
    	if distance < 0.3:
    	    score += 100.
    	elif distance <= 1.3:
    	    score += 100 - (distance-0.3)*100
    print ('Final score: %d'%(score/len(distance2points)))
    print ('Time used: %d'%timeUsed)

if __name__ == '__main__':
    main()
