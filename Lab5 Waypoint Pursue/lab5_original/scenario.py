import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from environs import Env
import lgsvl
from geometry_msgs.msg import Point
from rclpy.node import Node

DIS_THROSHOLD = 3
TIME_LIMIT = 300

EGO_POS = (-25.8, 0.0, 32.6)
EGO_ROT_Y = 249

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
    # load waypoints
    check_points = []
    with open("waypoints.csv") as fp:
        for line in fp:
            x, y = [float(v) for v in line.split(",")]
            check_points.append(CheckPoint(x, y))
    num_points = len(check_points)
    assert num_points > 0

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
    position_publisher = PositionPublisher()

    # wait for vehicle to start moving
    while not failed:
        # publish position
        publish_vehicle_position(position_publisher, ego)

        # run simulation
        sim.run(0.1)

        # check speed
        velocity = (
            ego.state.velocity.x ** 2
            + ego.state.velocity.y ** 2
            + ego.state.velocity.z ** 2
        ) ** 0.5
        if velocity >= 0.05:
            break

    # Track vehicle
    print("Start counting elapsed time.")
    point_index = 0
    elapsed_time = 0.0

    while not failed and point_index < num_points:
        # publish position
        publish_vehicle_position(position_publisher, ego)

        # run simulation
        sim.run(0.1)

        # check timeout
        elapsed_time += 0.1
        if elapsed_time > TIME_LIMIT:
            failed = True
            print("Timeout")
            break

        curr_point = check_points[point_index]

        # update distance to current and previous check point
        curr_point.update(ego.state.position.x, ego.state.position.z)
        if point_index > 0:
            check_points[point_index - 1].update(
                ego.state.position.x, ego.state.position.z
            )

        if curr_point.get_dis() < DIS_THROSHOLD:
            print("Waypoint {} passed".format(point_index + 1))
            point_index += 1

    # wait until the vehicle stops
    last_point = check_points[-1]

    while not failed:
        # publish position
        publish_vehicle_position(position_publisher, ego)

        # run simulation
        sim.run(0.1)

        # update distance to the last point
        last_point.update(ego.state.position.x, ego.state.position.z)

        # check speed
        velocity = (
            ego.state.velocity.x ** 2
            + ego.state.velocity.y ** 2
            + ego.state.velocity.z ** 2
        ) ** 0.5
        if velocity < 0.05:
            break

        elapsed_time += 0.1
        if elapsed_time > TIME_LIMIT:
            failed = True
            print("Timeout")
            break

    # judge
    print("==== JUDGE =====")

    if not failed:
        total_distance_score = 0
        for index, point in enumerate(check_points):
            dis = point.get_dis()
            if dis <= 0.5:
                score = 100
            elif dis <= 2.0:
                score = (2.0 - dis) * 20.0 / 1.5 + 80.0
            else:
                score = max((3.0 - dis) * 20.0 + 60.0, 0.0)
            total_distance_score += score
            print(
                "Waypoint {}\tdistance={:.2f}\tscore={:.2f}".format(
                    index + 1, dis, score
                )
            )

        distance_score = total_distance_score / num_points
        print("Distance score={:.2f}".format(distance_score))

        if elapsed_time <= 75:
            time_score = 100
        elif elapsed_time <= 100:
            time_score = 80
        elif elapsed_time <= 150:
            time_score = 60
        else:
            time_score = 0
            
        print("Elapsed time: {:.2f} seconds\ttime score={:.2f}".format(elapsed_time, time_score))

        final_score = distance_score * 0.8 + time_score * 0.2
        print("Final score: {:.2f}".format(final_score))
    else:
        print("Final score: 0.00")


def publish_vehicle_position(publisher, ego):
    vehicle_position = Point()
    vehicle_position.x = float(ego.state.position.x)
    vehicle_position.y = float(ego.state.position.y)
    vehicle_position.z = float(ego.state.position.z)
    publisher.pub.publish(vehicle_position)


if __name__ == "__main__":
    main()
