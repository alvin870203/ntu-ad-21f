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
LGSVL__MAP = env.str("LGSVL__MAP", lgsvl.wise.DefaultAssets.map_singlelaneroad)
LGSVL__VEHICLE_0 = env.str("LGSVL__VEHICLE_0", None)

if not LGSVL__VEHICLE_0:
    print('Please follow and instractions on document to retrieve the id of the vehicle.')
    exit(1)

INIT_X = -115
INIT_Y = 0
INIT_Z = -1.5

END_X = 110
END_Y = 0
END_Z = -1.5

ry = (END_Y-INIT_Y) / (END_X-INIT_X)
rz = (END_Z-INIT_Z) / (END_X-INIT_X)

CAR_ANGLE = 90
EGO_POS = (-120, INIT_Y, -1.5)
EGO_ROT_Y = CAR_ANGLE

NPC_POS = (INIT_X, INIT_Y, INIT_Z)
NPC_ROT_Y = CAR_ANGLE
END = 0


class BoxPublisher(Node):
    def __init__(self):
        super().__init__('BoxPublisher')
        self.pub = self.create_publisher(Detection3DArray,
                                         '/ground_truth/bounding_boxes', 10)

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

    # Add an NPC vehicle
    state = lgsvl.AgentState()
    state.transform.position = lgsvl.Vector(*NPC_POS)
    state.transform.rotation.y = NPC_ROT_Y
    npc = sim.add_agent(name="Sedan", agent_type=lgsvl.AgentType.NPC, state=state)
    
    waypoints = []
    speed = 10/3.6
    speed_range = (5/3.6, 100/3.6)
    acc_range = (5, 10)
    
    total_ttc = 0.
    waypoints_num = 0
    
    for i in range(INIT_X, 0, 2):
        acc = random.uniform(*acc_range)
        if 4*(speed**2)+8*acc < 0:
            end_speed = 0
        else:
            t = (-2*speed+(4*(speed**2)+4*acc)**0.5)/(2*acc)
            end_speed = speed+acc*t
        if end_speed < speed_range[0]:
            end_speed = speed_range[0]
        elif end_speed > speed_range[1]:
            end_speed = speed_range[1]
        speed = end_speed
        y = INIT_Y + (i-INIT_X) * ry
        z = INIT_Z + (i-INIT_X) * rz
        waypoints.append(lgsvl.DriveWaypoint(lgsvl.Vector(i, y, z), speed, angle=lgsvl.Vector(0, CAR_ANGLE, 0)))
    acc_range = (-10, -5)
    for i in range(0, END_X, 2):
        acc = random.uniform(*acc_range)
        if 4*(speed**2)+8*acc < 0:
            end_speed = 0
        else:
            t = (-2*speed+(4*(speed**2)+4*acc)**0.5)/(2*acc)
            end_speed = speed+acc*t
        if end_speed < speed_range[0]:
            end_speed = speed_range[0]
        elif end_speed > speed_range[1]:
            end_speed = speed_range[1]
        speed = end_speed
        y = INIT_Y + (i-INIT_X) * ry
        z = INIT_Z + (i-INIT_X) * rz
        waypoints.append(lgsvl.DriveWaypoint(lgsvl.Vector(i, y, z), speed, angle=lgsvl.Vector(0, CAR_ANGLE, 0)))
    obstacle_count = 0
    box_publisher = BoxPublisher()

    def on_collision(agent1, agent2, contact):
        nonlocal obstacle_count
        nonlocal sim

        if agent1 is not None and agent2 is not None:
            print("Collision occurred. Test failed")
            if contact is not None:
                print("{} collided with {} at {}".format(
                    agent1.name, agent2.name, contact))
            else:
                print("{} collided with {}".format(agent1.name, agent2.name))
            failed = True

        elif obstacle_count >= 2:
            print("Collision occurred. Test failed")
            if contact is not None:
                print("The car hits an obstacle at {}".format(contact))
            else:
                print("The car hits an obstacle")
            sim.stop()

        else:
            obstacle_count += 1
            
    def on_waypoint(agent, index):
        nonlocal total_ttc, waypoints_num
        npc_state = npc.state
        ego_state = ego.state
        ego_state.position.z = INIT_Z
        ego_state.rotation.y = CAR_ANGLE
        ego.state = ego_state
        msg = Detection3DArray()
        box = Detection3D()
        box.bbox.position.position.x = float(npc_state.position.x - ego_state.position.x)
        box.bbox.position.position.y = float(npc_state.position.y - ego_state.position.y)
        box.bbox.position.position.z = float(npc_state.position.z - ego_state.position.z)
        box.bbox.size.x = 4.56
        box.bbox.size.y = 2.08
        box.bbox.size.z = 1.35
        msg.detections.append(box)
        box_publisher.pub.publish(msg)
        
        velocity = ego_state.velocity.x
        distance = box.bbox.position.position.x
        if index > 9:
            ttc = distance / velocity
            if ttc < 0 or ttc > 999:
                print('Vehicle is not moving?')
            total_ttc += ttc
            waypoints_num += 1
            if index % 10 == 0:
                print("ckeckpoint %02d reached" % (index//10))
                print('Distance:', distance, ', TTC:', ttc, 'speed: ', velocity)
        if index == len(waypoints) - 1:
            sim.stop()
            print('judge finished')
            print('average TTC', (total_ttc / waypoints_num))

        

    ego.on_collision(on_collision)
    print(waypoints[0])
    npc.follow(waypoints, loop=False)
    npc.on_waypoint_reached(on_waypoint)
    # Starts simulation.
    print('judge started')

    sim.run()


if __name__ == '__main__':
    main()
