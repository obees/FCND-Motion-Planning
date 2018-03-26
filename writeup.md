## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation
##### motion_planning.py
---
> Main program that sets up the parameters for the simulation, creates the drone instance and calls the start() method to launch the programmed sequence of actions

```
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
```
---
> The MotionPlanning class inherits from Udacity Drone class and describes the drone's behavior in several methods

```
class MotionPlanning(Drone)
```
---
> **__init__(self, connection)**: Initialization of the drone object with connection parameter, sets default state and sets callbacks for managing position, velocity and state events

```
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
```
---
> **start(self)**: Method to start logging and to start the connection passed in the 'init' method
```
def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()
```
---
> **local_position_callback(self)**: manages the position events that are used in the takeoff and waypoint phases of the flight.

```
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
```
---
> **velocity_callback(self)**: manages the velocity events that are used in the landing phase of the flight.

```
def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()
```
---
> **state_callback(self)**: manages the state events that are used to trigger actions in the pre-flight and post-flight phases.

```
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
```
---
> **arming_transition(self)**: action to arm and take control over the drone before planning and takeoff
```
def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()
```

---
> **takeoff_transition(self)**: action to launch takeoff procedure that will target the  _self.target_position[2]_ position
```
def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])
```

---
> **waypoint_transition(self)**: action to manage the movements of the drone through the _self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])_ method that allows to set a target position and ask tell the drone to go there.
```
def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])
```

---
> **landing_transition(self)**: action to launch landing procedure of the drone at the current location
```
def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()
```

---
> **disarming_transition(self)**: action to launch disarming procedure of the drone after landing
```
def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()
```

---
> **manual_transition(self)**: action to launch manual procedure of the drone after disarming. It is the last action.
```
def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False
```

---
> **send_waypoints(self)**: method to send pre-calculated waypoints stored in the data list to the simulator
```
def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)
```

---
> **plan_path(self)**: method where all the flight plan is built as a list of waypoints that will be returned at the end of the method. The purpose is to build a representation of the environment and its constraints and to identify a flight plan allowing to go from start to goal locations.
```
def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values

        # TODO: set home position to (lon0, lat0, 0)

        # TODO: retrieve current global position

        # TODO: convert to current local position using global_to_local()

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center

        # Set goal as some arbitrary position on the grid
        grid_goal = (-north_offset + 10, -east_offset + 10)
        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
```
- The States class is an Enumeration allowing to describe the state the drone is in.
auto() generates automatically the ids for the states.

```
class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()
```
##### planning_utils.py
---
>  **create_grid(data, drone_altitude, safety_distance)**: utility method that allows to create a discretized grid representation of the obstacles data provided in the colliders.csv file. The output is a 2D matrix with a 1 in every cell where an obstacle is present and a 0 when space is free. The Origin of the grid is at [0,0] upper left corner.
north_min and east_min values allow to adapt obstacle coordinates from a center origin to a lower left origin
```
def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)
```
---
>  **Action(Enum)**: Action class describing allowed movements on the grid to build the path from start to goal. The cost of the action is also included to allow for optimization of the path through minimization of the cost
```
class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])
```
---
> **valid_actions(grid, current_node)**: Function to identify valid actions from the current position on the grid taking into account the constraints of the obstacles. Valid actions are valid movements in the context of the grid.
```
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)

    return valid_actions

```
---
>  **a_star(grid, h, start, goal)**: Function implementing a-star search algorithm on a grid to find a path from start to goal using a heuristic function for evaluating the cost of each path.
```
def a_star(grid, h, start, goal):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            # Get the new vertexes connected to the current vertex
            for a in valid_actions(grid, current_node):
                next_node = (current_node[0] + a.delta[0], current_node[1] + a.delta[1])
                new_cost = current_cost + a.cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node, a)

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost
```
---
>  **heuristic(position, goal_position)**: Function returning the cost left to get to the goal. It is used to complement the action cost and hence get a total cost for the action.
```
def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))
```


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.
> lat0 and lon0 are on the first line of the colliders.csv file. The line looks like this
```
lat0 37.792480, lon0 -122.397450
```
> To extract float lat0 and lon0
- Open file colliders.csv
- Read first line
- Remove end of line
- split by ',' to seperate lat0 and lon0
- suppress potential blank at the beginning of lat0 or lon0 string
- Split by ' ' lat0 and lon0 to separate label and values
- Select indice [1] to get values
- Cast to float values
- use self.set_home_position() method to set global home
```
with open('colliders.csv') as f:
        [lat0,lon0] = [float(a.lstrip(' ').split(' ')[1]) for a in f.readline().rstrip('\n').split(',')]

self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

- Global position is constructed from Drone class members for longitude, latitude and altitude
- Once global_position is set you get local position by using the utility method global_to_local()
```
global_position = (self._longitude, self._latitude, self._altitude)
current_local_pos = global_to_local(global_position, self.global_home)

```

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!
- the graph shares the coordinate system with the obstacles so no translation needed
```
grid_start = (int(current_local_pos[0]),int(current_local_pos[1]),int(current_local_pos[2]))
```
#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.
> The code below  allows to convert from geodetic coords (GPS) to local coords
```
grid_goal_global = (-122.398921, 37.797879, -20)
grid_goal_to_local = global_to_local(grid_goal_global, self.global_home)
grid_goal = (int(grid_goal_to_local[0]),int(grid_goal_to_local[1]),int(grid_goal_to_local[2]))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.
> A* adapted for graph is used here, the motion cost is calculated via heuristic since it fits measuring the distance between nodes of an edge. It's in the weight attribute of the added edge
```
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

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))
```

#### 6. Cull waypoints
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.
> To prune the path, the polygons methods are used to check is two node can connect. Relying on this information a pruning mechanism verifying every nodes pair is used
```
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

def can_connect(self,n1, n2, polygons):
      l = LineString([n1, n2])
      for p in polygons:
          if p.crosses(l) and p.height >= min(n1[2], n2[2]):
              return False
      return True
```


### Execute the flight
#### 1. Does it work?
It works!
Yes
### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.

# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.
