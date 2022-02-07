import random
from environs import Env

import lgsvl

env = Env()

LGSVL__SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
LGSVL__SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
LGSVL__AUTOPILOT_0_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
LGSVL__AUTOPILOT_0_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)
LGSVL__MAP = env.str("LGSVL__MAP", None)
LGSVL__VEHICLE_0 = env.str("LGSVL__VEHICLE_0", None)

INIT_Y = 0.0
EGO_POS = (-8, INIT_Y, 43)
EGO_ROT_Y = 249
NPC_POS_START = (-55, INIT_Y, 25)
NPC_POS_END = (-59, INIT_Y, 23)
NPC_ROT_Y = 249


def main():
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
    off_x = NPC_POS_END[0] - NPC_POS_START[0]
    off_y = NPC_POS_END[1] - NPC_POS_START[1]
    off_z = NPC_POS_END[2] - NPC_POS_START[2]
    ratio = random.uniform(0, 1)
    pos_x = NPC_POS_START[0] + off_x * ratio
    pos_y = NPC_POS_START[1] + off_y * ratio
    pos_z = NPC_POS_START[2] + off_z * ratio
    front_car_pos = (pos_x, pos_y, pos_z)

    state = lgsvl.AgentState()
    state.transform.position = lgsvl.Vector(*front_car_pos)
    state.transform.rotation.y = NPC_ROT_Y
    sim.add_agent(name="Sedan", agent_type=lgsvl.AgentType.NPC, state=state)

    failed = False
    obstacle_count = 0

    def on_collision(agent1, agent2, contact):
        nonlocal failed
        nonlocal obstacle_count

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
            failed = True

        else:
            obstacle_count += 1

    ego.on_collision(on_collision)

    # Starts simulation.
    print('judge started')

    while True:
        # first stage
        while not failed:
            sim.run(time_limit=0.1)
            s = ego.state
            velocity = (s.velocity.x**2 + s.velocity.y**2 +
                        s.velocity.z**2)**0.5

            if velocity * 3.6 >= 40:
                print('1st stage passed')
                break

        # second stage
        while not failed:
            sim.run(time_limit=0.1)
            s = ego.state
            velocity = (s.velocity.x**2 + s.velocity.y**2 +
                        s.velocity.z**2)**0.5

            if velocity * 3.6 <= 5:
                car_dist = (((s.position.x - front_car_pos[0])**2 +
                             (s.position.z - front_car_pos[2])**2)**0.5)
                dist = max(car_dist - 4.8, 0)
                print('2nd stage passed')
                print('Vehicle stopped with distance: %0.3f' % dist)
                break

        break

    sim.stop()
    print('judge finished')


if __name__ == '__main__':
    main()
