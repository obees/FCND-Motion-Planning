from enum import Enum
from queue import PriorityQueue
import numpy as np
import math
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

# A* for graphs
def a_star_graph(graph, heuristic, start, goal):

    path = []
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
            for next_node in graph[current_node]:
                cost = graph.edges[current_node,next_node]["weight"]
                new_cost = current_cost + cost + heuristic(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))

                    branch[next_node] = (new_cost, current_node)

    path = []
    path_cost = 0
    if found:

        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        path.append(n)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('Path NOT Found ... Run again to generate new random nodes ...')
    return path[::-1], path_cost

class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]

    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)



def extract_polygons(data, safety_distance):

    polygons = []
    marge = safety_distance
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        obstacle = [int(north - d_north - marge), int(north + d_north + marge), int(east - d_east - marge), int(east + d_east + marge)]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]

        # TODO: Compute the height of the polygon
        height = alt + d_alt + marge

        p = Poly(corners, height)
        polygons.append(p)

    return polygons
class Sampler:

    def __init__(self, data, target_altitude, safety_distance):
        self._polygons = extract_polygons(data, safety_distance)
        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        self._zmin = target_altitude - 1
        # limit z-axis
        self._zmax = target_altitude + 1

        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')

    def sample(self, num_samples, grid_start, grid_goal):
        """Implemented with a k-d tree for efficiency."""
        xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
        samples = list(zip(xvals, yvals, zvals))
        samples.insert(0,grid_start)
        samples.append(grid_goal)
        pts = []
        for s in samples:
            _, idx = self._tree.query(np.array([s[0], s[1]]).reshape(1, -1))
            p = self._polygons[int(idx)]
            if not p.contains(s) or p.height < s[2]:
                pts.append(s)
        return pts

    @property
    def polygons(self):
        return self._polygons
