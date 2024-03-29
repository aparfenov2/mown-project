# A* path planning using kinematic state space of car.
# pp_02_b_car_statespace_astar_solution
# (c) Claus Brenner, 16 JAN 2014
import numpy as np
from math import sqrt, sin, cos, pi, floor, radians, copysign
from heapq import heappush, heappop
import traceback

from numpy.core.einsumfunc import _parse_possible_contraction
# import gui
# import common

# The world extents in units.
world_extents = (100, 75)

# The obstacle map.
# Obstacle = 255, free space = 0.
world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_cells = None

# Switch which determines if visited cells shall be drawn in the GUI.
show_visited_cells = True

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

# # Functions for GUI functionality.
# def add_obstacle(pos):
#     common.set_obstacle(world_obstacles, pos, True)
#     common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
#                            show_visited_cells)
# def remove_obstacle(pos):
#     common.set_obstacle(world_obstacles, pos, False)
#     common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
#                            show_visited_cells)
# def clear_obstacles():
#     global world_obstacles
#     world_obstacles = np.zeros(world_extents, dtype=np.uint8)
#     update_callback()
# def toggle_visited_cells():
#     global show_visited_cells
#     show_visited_cells = not show_visited_cells
#     common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
#                            show_visited_cells)
# def toggle_movement():
#     global max_movement_id
#     max_movement_id = 9 - max_movement_id  # Toggles between 3 and 6.
#     update_callback()
# def update_callback(pos = None):
#     # Call path planning algorithm.
#     start, goal = gui.get_start_goal()
#     if not (start==None or goal==None):
#         global optimal_path
#         global visited_cells
#         try:
#             optimal_path, visited_cells = \
#                 astar_statespace(start, goal, world_obstacles)
#         except Exception as e:
#             print traceback.print_exc()
    # # Draw new background.
    # common.draw_background(gui, world_obstacles, visited_cells, optimal_path,
    #                        show_visited_cells)

# --------------------------------------------------------------------------
# Helper class for curved segments.
# --------------------------------------------------------------------------
class LineSegment:
    @staticmethod
    def end_pose(start_pose, direction, length):
        """Returns end pose, given start pose, curvature and length."""
        x, y, theta = start_pose
            # Linear movement.
        x += length * cos(direction)
        y += length * sin(direction)
        return (x, y, direction)

    @staticmethod
    def segment_points(start_pose, direction, length, delta_length):
        """Return points of segment, at delta_length intervals."""
        l = 0.0
        delta_length = copysign(delta_length, length)
        points = []
        while abs(l) < abs(length):
            points.append(LineSegment.end_pose(start_pose, direction, l)[0:2])
            l += delta_length
        return points


class CurveSegment:
    @staticmethod
    def end_pose(start_pose, curvature, length):
        """Returns end pose, given start pose, curvature and length."""
        x, y, theta = start_pose
        if curvature == 0.0:
            # Linear movement.
            x += length * cos(theta)
            y += length * sin(theta)
            return (x, y, theta)
        else:
            # Curve segment of radius 1/curvature.
            tx = cos(theta)
            ty = sin(theta)
            radius = 1.0 / curvature
            xc = x - radius * ty  # Center of circle.
            yc = y + radius * tx
            angle = length / radius
            cosa = cos(angle)
            sina = sin(angle)
            nx = xc + radius * (cosa * ty + sina * tx)
            ny = yc + radius * (sina * ty - cosa * tx)
            ntheta = (theta + angle + pi) % (2*pi) - pi
            return (nx, ny, ntheta)

    @staticmethod
    def segment_points(start_pose, curvature, length, delta_length):
        """Return points of segment, at delta_length intervals."""
        l = 0.0
        delta_length = copysign(delta_length, length)
        points = []
        while abs(l) < abs(length):
            points.append(CurveSegment.end_pose(start_pose, curvature, l)[0:2])
            l += delta_length
        return points


class CurveGenerator(object):
    def __init__(self) -> None:
        super().__init__()

        movements = []

        for i in [1, 2]:
            # for direction in [0, np.pi / 4.0, np.pi / -4.0, np.pi / 2.0, np.pi / -2.0, np.pi * 3.0/ 4.0, np.pi * 3.0/ -4.0, np.pi]:
            for curvature in np.linspace(-0.25, 0.25, num=11):
                movements.append((curvature, i))

        for i in [5]:
            # for direction in [0, np.pi / 4.0, np.pi / -4.0, np.pi / 2.0, np.pi / -2.0, np.pi * 3.0/ 4.0, np.pi * 3.0/ -4.0, np.pi]:
            for curvature in np.linspace(-0.1, 0.1, num=11):
                movements.append((curvature, i))

        self.movements = movements

    def generate(self, pose):
        states = list()
        for i in range(len(self.movements)):
            curvature, length = self.movements[i]
            new_pose = CurveSegment.end_pose(pose, curvature, length)
            states.append((new_pose, length, i))

        return states

    def segment_points(self, start_pose, segment_points_args, delta_length):
        curvature, length = self.movements[segment_points_args]
        return CurveSegment.segment_points(start_pose, curvature, length, delta_length)


class LinesGenerator(object):
    def __init__(self, grid_size) -> None:
        super().__init__()

        movements = []

        for i in [grid_size, 2 * grid_size]:
            # for direction in [0, np.pi / 4.0, np.pi / -4.0, np.pi / 2.0, np.pi / -2.0, np.pi * 3.0/ 4.0, np.pi * 3.0/ -4.0, np.pi]:
            for direction in np.linspace(-np.pi / 4, np.pi / 4, num=5):
                movements.append((direction, i))

        for i in [5 * grid_size]:
            # for direction in [0, np.pi / 4.0, np.pi / -4.0, np.pi / 2.0, np.pi / -2.0, np.pi * 3.0/ 4.0, np.pi * 3.0/ -4.0, np.pi]:
            for direction in np.linspace(-np.pi / 6, np.pi / 6, num=3):
                movements.append((direction, i))

        self.movements = movements

    def generate(self, pose):
        states = list()
        for i in range(len(self.movements)):
            curvature, length = self.movements[i]
            new_pose = CurveSegment.end_pose(pose, curvature, length)
            states.append((new_pose, length, i))

        return states

    def segment_points(self, start_pose, segment_points_args, delta_length):
        curvature, length = self.movements[segment_points_args]
        return CurveSegment.segment_points(start_pose, curvature, length, delta_length)


class GridGenerator(object):
    def __init__(self, raster) -> None:
        super().__init__()

        self.raster = raster

        self.movements = [
            (raster, 0.0),
            (raster, -raster),
            (0.0, -raster),
            (-raster, -raster),
            (-raster, 0.0),
            (-raster, raster),
            (0.0, raster),
            (raster, raster)
        ]

    def generate(self, pose):
        saved_pose = pose
        pose = self.floor_pose(pose)
        states = list()
        for move in self.movements:
                x, y, _ = pose
                mx, my = move

                new_pose = (x + mx, y + my, 0.0)
                length = distance(saved_pose, new_pose)
                states.append((new_pose, length, (mx, my)))

        return states

    def floor_pose(self, pose):
        pos_raster = self.raster
        xi = pos_raster * floor(pose[0] / pos_raster)
        yi = pos_raster * floor(pose[1] / pos_raster)
        return (xi, yi, 0.0)


    def segment_points(self, *args, **kwargs):
        return LineSegment.segment_points(*args, **kwargs)

    def segment_points(self, start_pose, segment_points_args, delta_length):
        points = []
        points.append(start_pose[:2])

        x, y, _ = start_pose
        mx, my = segment_points_args
        new_pose = (x + mx, y + my)
        points.append(new_pose)
        
        return points

# --------------------------------------------------------------------------
# Exploration of car's kinematic state space.
# --------------------------------------------------------------------------

# Allowed movements. These are given as tuples: (curvature, length).
# The list contains forward and backward movements.
movements = [(1.0/10, 5.0), (0.0, 5.0), (-1.0/10, 5.0),
             (1.0/10, -5.0), (0.0, -5.0), (-1.0/10, -5.0)]
max_movement_id = 3  # 3 for forward only, 6 for forward and backward.

# Helper functions.
def distance(p, q):
    """Return Euclidean distance between two points."""
    return sqrt((p[0]-q[0])**2 + (p[1]-q[1])**2)

def states_close(p, q):
    """Checks if two poses (x, y, heading) are the same within a tolerance."""
    # d_angle = abs((p[2]-q[2]+pi) % (2*pi) - pi)
    # For the sake of simplicity, tolerances are hardcoded here:
    # 15 degrees for the heading angle, 2.0 for the position.
    # return d_angle < radians(15.) and distance(p, q) <= 1.0
    return distance(p, q) <= 1.0

def pose_index(pose):
    """Given a pose, returns a discrete version (a triple index)."""
    # Again, raster sizes are hardcoded here for simplicity.
    pos_raster = 1.0
    heading_raster = radians(10.)
    xi = int(floor(pose[0] / pos_raster))
    yi = int(floor(pose[1] / pos_raster))
    ti = int(floor(pose[2] / heading_raster))
    return (xi, yi, ti)


class AstarPathPlanner(object):
    def __init__(self, generator):
        
        self.STOP_SEARCH_FRONT_LEN = 500000
        self.state_generator = generator

    def astar_statespace(self, start_pose, goal_pose):
        """
        Try to find a sequence of curve segments from start to goal.
        """

        # Init front: the only element is the start pose.
        # Each tuple contains:
        # (total_cost, cost, pose, previous_pose, move_index).
        front = [ (distance(start_pose, goal_pose), 0.0, start_pose,
                None, None) ]

        # In the beginning, no cell has been visited.
        # extents = obstacles.shape
        # visited_cells = np.zeros(extents, dtype=np.float32)


        # Also, no states have been generated.
        generated_states = {}

        weight_table = {}

        while front:
            # Stop search if the front gets too large.
            if len(front) > self.STOP_SEARCH_FRONT_LEN:
                print("Timeout.")
                break

            # Pop smallest item from heap.
            total_cost, cost, pose, previous_pose, move = heappop(front)

            # Compute discrete index.
            # CHANGE 02_b: compute a discrete pose index pose_idx from pose,
            #   using the function pose_index() above.
            pose_idx = pose_index(pose)

            # Check if this has been visited already.
            # CHANGE 02_b: check if pose_index is in generated_states already,
            #   and if so, skip the rest of the loop. (This is the point where
            #   we prevent exponential growth.)

            if pose_index in generated_states.keys():
                previous_pose, move, saved_cost = generated_states[pose_idx][2]
                if cost < saved_cost:
                    generated_states[pose_idx] = (previous_pose, move, cost)
                continue

            weight_table[pose[:2]] = cost


            # Mark visited_cell which encloses this pose.
            # visited_cells[int(pose[0]), int(pose[1])] = cost

            # Enter into visited states, also use this to remember where we
            # came from and which move we used.
            # CHANGE 02_b: Change the following line so that the index is the
            #   discrete pose instead of the continuous pose.
            generated_states[pose_idx] = (previous_pose, move, cost)

            # Check if we have (approximately) reached the goal.
            if states_close(pose, goal_pose):
                break  # Finished!

            # Check all possible movements.
            new_states = self.state_generator.generate(pose)

            for state in new_states:
                new_pose, length, segment_points_args = state
                new_cost = cost + abs(length)
                total_cost = new_cost + 5 * distance(new_pose, goal_pose)
                heappush(front, (total_cost, new_cost, new_pose, pose, segment_points_args))

            # for i in range(len(self.movements)):
            #     curvature, length = self.movements[i]

            #     # Determine new pose and check bounds.
            #     # new_pose = CurveSegment.end_pose(pose, curvature, length)
            #     new_pose = LineSegment.end_pose(pose, curvature, length)
            #     # if not (0 <= new_pose[0] < extents[0] and \
            #     #         0 <= new_pose[1] < extents[1]):
            #     #     continue

            #     # Add to front if there is no obstacle.
            #     # if not obstacles[(int(new_pose[0]), int(new_pose[1]))] == 255:
            #     new_cost = cost + abs(length)
            #     total_cost = new_cost + distance(new_pose, goal_pose)
            #     heappush(front, (total_cost, new_cost, new_pose, pose, i))

        # Reconstruct path, starting from goal.
        # (Take this part of the code as is. Note it is different from the
        #  pp_02_a code.)
        # if states_close(pose, goal_pose):
        #     path = []
        #     path.append(pose[0:2])
        #     pose, segment_points_args, _ = generated_states[pose_index(pose)]
        #     while pose:
        #         # points = CurveSegment.segment_points(pose,
        #         #     self.movements[move][0], self.movements[move][1], 0.50)
        #         points = self.state_generator.segment_points(
        #             pose,
        #             segment_points_args, 
        #             0.50
        #         )
        #         path.extend(reversed(points))
        #         pose, move, _ = generated_states[pose_index(pose)]
        #     path.reverse()

        # for key, value in generated_states.items():
        #     pos = value[0]
        #     if pos is not None:

        #         plt.scatter(pos[0], pos[1])


        if states_close(pose, goal_pose):
            path = []
            path.append(pose[0:2])
            pose, segment_points_args, _ = generated_states[pose_index(pose)]
            while pose:
                # points = CurveSegment.segment_points(pose,
                #     self.movements[move][0], self.movements[move][1], 0.50)
                points = self.state_generator.segment_points(
                    pose,
                    segment_points_args, 
                    0.2
                )
                path.extend(reversed(points))
                pose, segment_points_args, _ = generated_states[pose_index(pose)]
            path.reverse()
        else:
            path = []

        return path # , visited_cells


def draw_line_segments(): 
    movements = []

    for i in [1.0, 5.0]:
        # for direction in [0, np.pi / 4.0, np.pi / -4.0, np.pi / 2.0, np.pi / -2.0, np.pi * 3.0/ 4.0, np.pi * 3.0/ -4.0, np.pi]:
        for direction in np.linspace(0.0, 2.0 * np.pi, num=20):
            movements.append((i, direction))

    for move in movements:
        i, direction = move
        x, y = [0], [0]
        new_pose = LineSegment.end_pose((0.0, 0.0, 0.0), direction, i)
        x.append(new_pose[0])
        y.append(new_pose[1])

        plt.plot(x, y)
        plt.plot(x[-1], y[-1], 'o')  

    plt.legend()
    plt.show()


def draw_movements():


    movements = []

    for i in range(11):
        for l in [10.0, 100.0]:
            c = (i - 5) / l
            movements.append((c, 5.0))
            movements.append((c, -5.0))
            movements.append((c, 1.0))
            movements.append((c, -1.0))
    
    import matplotlib.pyplot as plt   
    import matplotlib

    matplotlib.use('TkAgg')

    plt.plot([0], [0], 'o')

    for move in movements:
        curvature, length = move
        dlenght = length / 10.0

        for i in [1.0, 2.0, 5.0]:
            # for direction in [0, np.pi / 4.0, np.pi / -4.0, np.pi / 2.0, np.pi / -2.0, np.pi * 3.0/ 4.0, np.pi * 3.0/ -4.0, np.pi]:
            for direction in np.linspace(0.0, 2.0 * np.pi, num=20):
                x, y = [0], [0]
                new_pose = LineSegment.end_pose((0.0, 0.0, 0.0), direction, i)
                x.append(new_pose[0])
                y.append(new_pose[1])

                plt.plot(x, y)
                plt.plot(x[-1], y[-1], 'o')

    
    plt.legend()
    plt.show()
        

def find_path():
    start_pos = (0.0, 0.0, 0.0)
    goal_pos = (-20.0, 110.0, 0.0)
    
    lg = CurveGenerator()

    planner = AstarPathPlanner(lg)
    path = planner.astar_statespace(start_pos, goal_pos)

    import matplotlib.pyplot as plt   
    import matplotlib

    matplotlib.use('TkAgg')

    generated_states = lg.generate((0.0, 0.0, 0.0))

    poses = [state[0] for state in generated_states]

    plt.scatter([p[0] for p in poses], [p[1] for p in poses], label='Trajectory')
    
    plt.legend()
    plt.show()

    plt.plot([p[0] for p in path], [p[1] for p in path], label='Trajectory')
    

    plt.legend()
    plt.show()



# Main program.
if __name__ == '__main__':
    # Link functions to buttons.
    import matplotlib.pyplot as plt   
    import matplotlib
    matplotlib.use('TkAgg')
    find_path()
    # movements = [(1.0/10, 5.0), (2.0/10, 5.0),(0.0, 5.0), (0.0, 1.0), (-2.0/10, 5.0), (-1.0/10, 5.0),
    #              (1.0/10, -5.0), (0.0, -5.0), (0.0, -1.0), (-1.0/10, -5.0)]
    # draw_line_segments()

    # callbacks = {"update": update_callback,
    #              "button_1_press": add_obstacle,
    #              "button_1_drag": add_obstacle,
    #              "button_1_release": update_callback,
    #              "button_2_press": remove_obstacle,
    #              "button_2_drag": remove_obstacle,
    #              "button_2_release": update_callback,
    #              "button_3_press": remove_obstacle,
    #              "button_3_drag": remove_obstacle,
    #              "button_3_release": update_callback,
    #              }
    # # Extra buttons.
    # buttons = [("Forward/FW+BW", toggle_movement),
    #            ("Clear", clear_obstacles),
    #            ("Show Visited", toggle_visited_cells)]

    # # Init GUI.
    # gui = gui.GUI(world_extents, 8, callbacks,
    #               buttons, "oriented",
    #               "Car state space A*.")

    # # Start GUI main loop.
    # gui.run()