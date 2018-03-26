import argparse
import time
import msgpack
import numpy as np
import networkx as nx
import numpy.linalg as LA
from enum import Enum, auto
from planning_utils import a_star_graph, heuristic, Sampler
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from udacidrone.frame_utils import local_to_global
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue
from sklearn.neighbors import KDTree

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        print('self.target_position[2]'.format(self.target_position[2]))
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        print('self.waypoints:{}'.format(self.waypoints))
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def can_connect(self,n1, n2, polygons):
        l = LineString([n1, n2])
        for p in polygons:
            if p.crosses(l) and p.height >= min(n1[2], n2[2]):
                return False
        return True

    def create_graph(self,nodes, k, polygons):
        g = nx.Graph()
        tree = KDTree(nodes)
        for n1 in nodes:
            # for each node connect try to connect to k nearest nodes
            idxs = tree.query([n1], k, return_distance=False)[0]

            for idx in idxs:
                n2 = nodes[idx]
                if n2 == n1:
                    continue

                if self.can_connect(n1, n2, polygons):
                    heuristic_val = heuristic(n1,n2)
                    g.add_edge(n1, n2, weight=heuristic_val)
        return g

    def prune_path(self, path, polygons):

        pruned_path = [p for p in path]
        # TODO: prune the path!

        i = 0
        while i < len(pruned_path)-2:
            p1 = pruned_path[i]
            j=i+2
            connected_j = 0
            while j < len(pruned_path):
                p3 = pruned_path[j]
                if self.can_connect(p1, p3, polygons):
                    connected_j = j
                j = j + 1
            if connected_j > 0:
                for k in range(i+1, connected_j):
                    pruned_path.remove(pruned_path[i+1])
            i = i + 1

        return pruned_path

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 19
        SAFETY_DISTANCE = 5

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            [lat0,lon0] = [float(a.lstrip(' ').split(' ')[1]) for a in f.readline().rstrip('\n').split(',')]

        # TODO: set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        global_position = (self._longitude, self._latitude, self._altitude)

        # TODO: convert to current local position using global_to_local()
        current_local_pos = global_to_local(global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # TODO: convert start position to current position rather than map center
        grid_start = (int(current_local_pos[0]),int(current_local_pos[1]),int(current_local_pos[2]))

        # TODO: adapt to set goal as latitude / longitude position and convert
        grid_goal_global = (-122.398921, 37.797879, -20)
        grid_goal_to_local = global_to_local(grid_goal_global, self.global_home)
        grid_goal = (int(grid_goal_to_local[0]),int(grid_goal_to_local[1]),int(grid_goal_to_local[2]))


        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)

        print('Local Start and Goal: ', grid_start, grid_goal)

        # Using graph approach
        # Sampling environment initialization with obstacles data
        # sampling class using a k-d tree adapted to support TARGET_ALTITUDE, SAFETY_DISTANCE, grid_start, grid_goal
        sampler = Sampler(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        # Retrieve polygons objects list representing the obstacles
        # Giving access to polygon.contains and polygon.crosses methods
        polygons = sampler._polygons

        # sampling random_points_quantity points and removing
        # ones conflicting with obstacles.
        # Adding grid_start and grid_goal allows to verify against obstacle collision early
        random_points_quantity = 400
        print('Generating {} random nodes ...'.format(random_points_quantity))
        nodes = sampler.sample(random_points_quantity, grid_start, grid_goal)

        # create graph from possible states, obstacle polygons, and edges to 10 closest states
        print("Creating a graph from random nodes ...")
        g = self.create_graph(nodes, 10, polygons)

        #Search a path using graph adapted A* algorithm
        print("Searching for a path from start to goal in graph ...")
        path, cost = a_star_graph(g, heuristic, grid_start, grid_goal)

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        path = self.prune_path(path, polygons)

        # Convert path to waypoints skipping grid_start
        waypoints = [[int(p[0]), int(p[1]), int(p[2]), 0] for p in path[1:]]

        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim
        # Abort mission if path to goal not found
        if len(path)>0:
            self.send_waypoints()
        else:
            self.disarming_transition()
    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
