import random
from datetime import datetime
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from environs import Env
import lgsvl
from geometry_msgs.msg import Point
from rclpy.node import Node

DIS_THROSHOLD = 3
TIME_LIMIT = 300

EGO_POS = (35.66, 0.00, 24.11)
# EGO_POS = (35.50, 0.0, 35.00)
EGO_ROT_Y = 70
# EGO_ROT_Y = 250

END_POS = (34.90, 0.0, 36.00)


env = Env()
LGSVL__SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
LGSVL__SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
LGSVL__AUTOPILOT_0_HOST = env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
LGSVL__AUTOPILOT_0_PORT = env.int("LGSVL__AUTOPILOT_0_PORT", 9090)
LGSVL__MAP = env.str("LGSVL__MAP", "Shalun")
LGSVL__VEHICLE_0 = env.str("LGSVL__VEHICLE_0", "Lexus2016RXHybrid")


class PositionPublisher(Node):
    def __init__(self):
        super().__init__("PositionPublisher")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            Point, "/ground_truth/vehicle_position", qos_profile
        )


# checkPoints
class CheckPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.dis = None

    def update(self, x, y):
        dis = (((x - self.x) ** 2) + ((y - self.y) ** 2)) ** 0.5
        if self.dis is None or dis < self.dis:
            self.dis = dis

    def get_dis(self):
        return self.dis


def main():
    # initialize rclpy
    rclpy.init()

    # Initialize simulator
    sim = lgsvl.Simulator(address=LGSVL__SIMULATOR_HOST, port=LGSVL__SIMULATOR_PORT)
    if sim.current_scene == LGSVL__MAP:
        sim.reset()
    else:
        sim.load(scene=LGSVL__MAP, seed=650387)

    # Chooses ego vehicles.
    state = lgsvl.AgentState()
    state.transform.position = lgsvl.Vector(*EGO_POS)
    state.transform.rotation.y = EGO_ROT_Y
    ego = sim.add_agent(
        name=LGSVL__VEHICLE_0, agent_type=lgsvl.AgentType.EGO, state=state
    )
    ego.connect_bridge(LGSVL__AUTOPILOT_0_HOST, LGSVL__AUTOPILOT_0_PORT)

    state = lgsvl.AgentState()
    speed = random.uniform(5.0, 8.0)
    waypoints = load_waypoints("npc1.csv", speed)
    state.transform.position = waypoints[0].position
    state.transform.rotation = waypoints[0].angle
    npc1 = sim.add_agent(name="Sedan", agent_type=lgsvl.AgentType.NPC, state=state)
    npc1.follow(waypoints, loop=True)

    state = lgsvl.AgentState()
    speed = random.uniform(6.0, 10.0)
    waypoints = load_waypoints("npc2.csv", 5.0)
    state.transform.position = waypoints[0].position
    state.transform.rotation = waypoints[0].angle
    npc2 = sim.add_agent(name="Sedan", agent_type=lgsvl.AgentType.NPC, state=state)
    npc2.follow(waypoints, loop=True)

    failed = False
    obstacle_count = 0

    def on_collision(agent1, agent2, contact):
        nonlocal failed
        nonlocal obstacle_count

        if agent1 is not None and agent2 is not None:
            print("Collision occurred. Test failed")
            if contact is not None:
                print(
                    "{} collided with {} at {}".format(
                        agent1.name, agent2.name, contact
                    )
                )
            else:
                print("{} collided with {}".format(agent1.name, agent2.name))
            failed = True

        elif obstacle_count >= 0:
            print("Collision occurred. Test failed")
            if contact is not None:
                print("The car hits an obstacle at {}".format(contact))
            else:
                print("The car hits an obstacle")
            failed = True

        else:
            obstacle_count += 1

    ego.on_collision(on_collision)

    # create position publisher
    fp = open("{}.csv".format(datetime.now().isoformat()), "w")
    position_publisher = PositionPublisher()
    step = 0

    while not failed:
        # publish position
        publish_vehicle_position(position_publisher, ego)
        step += 1

        # run simulation
        sim.run(0.1)

        # save position
        line = "{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}\n".format(
            ego.state.position.x,
            ego.state.position.y,
            ego.state.position.z,
            ego.state.velocity.x,
            ego.state.velocity.y,
            ego.state.velocity.z,
            ego.state.rotation.y,
        )
        fp.write(line)

        distance = (
            (ego.state.position.x - END_POS[0]) ** 2
            + (ego.state.position.z - END_POS[2]) ** 2
        ) ** 0.5
        speed = (ego.state.velocity.x ** 2 + ego.state.velocity.z ** 2) ** 0.5

        print(f"{distance=}; {speed=}")

        if distance <= 2.0 and speed <= 0.5:
            print("Finished")
            break


def publish_vehicle_position(publisher, ego):
    vehicle_position = Point()
    vehicle_position.x = float(ego.state.position.x)
    vehicle_position.y = float(ego.state.position.y)
    vehicle_position.z = float(ego.state.position.z)
    publisher.pub.publish(vehicle_position)


def load_waypoints(path, vel: float):
    waypoints = list()

    with open(path, "r") as fp:
        for line in fp:
            (x, _, z, rot_y) = line.split(",")
            x = float(x)
            z = float(z)
            rot_y = float(rot_y)
            pos = lgsvl.Vector(x, 0.0, z)
            rot = lgsvl.Vector(0.0, rot_y, 0.00)
            waypoints.append(lgsvl.DriveWaypoint(pos, vel, angle=rot))

    return waypoints


if __name__ == "__main__":
    main()
