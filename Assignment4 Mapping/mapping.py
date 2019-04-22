#!/usr/bin/env python3

"""
    # {Rui Qu}
    # {19930307-T550}
    # {rqu@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs, sqrt

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        obstacles = []
        xl=[]
        yl=[]

        robot_x=pose.pose.position.x - origin.position.x
        robot_y=pose.pose.position.y - origin.position.y
        
        for i in range(len(scan.ranges)):
            if scan.ranges[i] > scan.range_min and scan.ranges[i] < scan.range_max:
                bearing = scan.angle_min +  i* scan.angle_increment
                x = scan.ranges[i] * cos(scan.angle_min +  i* scan.angle_increment + robot_yaw ) + robot_x
                y = scan.ranges[i] * sin(scan.angle_min +  i* scan.angle_increment + robot_yaw ) + robot_y

                x = int(x / resolution)
                y = int(y / resolution)

                xl.append(x)
                yl.append(y)
                obstacles.append((x, y))
                
                free_cells = self.raytrace([int((pose.pose.position.x - origin.position.x) / resolution), int((pose.pose.position.y - origin.position.y) / resolution)], [x, y])
                for free_cell in free_cells:
                    (cell_x, cell_y) = free_cell
                    self.add_to_map(grid_map, cell_x, cell_y, self.free_space)     

        for obstacle in obstacles:
            (x, y) = obstacle
            self.add_to_map(grid_map, x, y, self.occupied_space)

        """
        For C only!
        Fill in the update correctly below.
        """ 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min(xl)
        # The minimum y index in 'grid_map' that has been updated
        update.y = min(yl)
        # Maximum x index - minimum x index + 1
        update.width = max(xl) - min(xl) + 1
        # Maximum y index - minimum y index + 1
        update.height = max(yl) - min(yl) + 1
        # The map data inside the rectangle, in h-major order.
        
        update.data = []
        for h in range(update.height):
            for w in range(update.width):
                update.data.append(grid_map.__getitem__([h, w]))

        # Return the updated map together with only the
        # part of the map that has been updated
        return grid_map, update

    def expansion(self,point):
        #extend a point to an area with radius of self.radius
        x=point[0]
        y=point[1]
        points=[]
        points.append([x+4,y])
        points.append([x-4,y])
        points.append([x,y+4])
        points.append([x,y-4])
        for i in range(4):
            for j in range(4):
                points.append([x+i,y+j])
                points.append([x-i,y-j])
                points.append([x+i,y-j])
                points.append([x-i,y+j])
        points.remove([x+3,y+3])
        points.remove([x+3,y-3])
        points.remove([x-3,y+3])
        points.remove([x-3,y-3])
        
        return points        

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map mean`s that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.

        

        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """
        width = grid_map.get_width()
        height = grid_map.get_height()
        origin = grid_map.get_origin()
        for w in range(width):
            for h in range(height):
                x=int(origin.position.x+w)
                y=int(origin.position.y+h)
                if self.is_in_bounds(grid_map,x,y):
                    if grid_map[x,y]==self.occupied_space:
                        points =self.expansion([x,y])
                        for point in points:
                            if grid_map[point[0],point[1]]!= self.occupied_space:
                                    self.add_to_map(grid_map,point[0],point[1],self.c_space)
        # Return the inflated map
        return grid_map
