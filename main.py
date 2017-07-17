import matplotlib.pyplot as plt
from Geom import *

# Constants for the field/simulation
FIELD_LENGTH = 9
FIELD_WIDTH = 6
CENTER_CIRCLE_RADIUS = 0.5
GOAL_WIDTH = 1
GOAL_DEPTH = 0.02

ROBOT_RADIUS = 0.09
AVOID_THRESHOLD = 0.05
TOTAL_AVOID_DIST = 2 * ROBOT_RADIUS + AVOID_THRESHOLD
NEW_POINT_BUFFER = 0.02
TOTAL_AVOID_DIST_WITH_BUFFER = TOTAL_AVOID_DIST + NEW_POINT_BUFFER

ROBOT_AVOID_DIST = 2 * ROBOT_RADIUS + 0.1

robots = [Circle(Point(0, 0), ROBOT_AVOID_DIST), Circle(Point(0, -0.2), ROBOT_AVOID_DIST), Circle(Point(-2, -0.0), ROBOT_AVOID_DIST),
          Circle(Point(-2, -0.4), ROBOT_AVOID_DIST), Circle(Point(2, -0.2), ROBOT_AVOID_DIST),
          Circle(Point(-2.7, -0.0), ROBOT_AVOID_DIST), Circle(Point(-2.7, -0.3), ROBOT_AVOID_DIST), Circle(Point(-2.7, 0.3), ROBOT_AVOID_DIST),
          Circle(Point(-3, -0.6), ROBOT_AVOID_DIST), Circle(Point(-3.4, -0.7), ROBOT_AVOID_DIST),
          Circle(Point(-3.5, -0.2), ROBOT_AVOID_DIST)]

start_ = Point(-3, 0)
target_ = Point(3, 0)

# define the modes for the planning algorithm
MODE_LEFT = "MODE_LEFT"
MODE_RIGHT = "MODE_RIGHT"
MODE_BOTH = "MODE_BOTH"

# THIS IS V2, the right left mode version
def straight_line_planner(start, target, obstacles, mode):
    first_collision = get_first_collision(start, target, obstacles)
    if first_collision is None:
        return [target]

    if mode is MODE_LEFT:
        obstacle_group = get_group_of_points(first_collision, obstacles)
        left_perp_point = None
        left_perp_point_dist = -1

        # perp() function returns to the clockwise side
        for ob in obstacle_group:
            point = ob.origin - (target - start).perp().norm(ob.radius + NEW_POINT_BUFFER)
            dist = Util.dist_point_to_line(point, Line(start, target))

            # value is negative is point is on "left" of line looking from start to target
            if not Util.point_is_to_right_of_line(start, target, point) and (
                    left_perp_point is None or dist > left_perp_point_dist) \
                    and Util.closest_point_on_line(Line(start, target), point) != start \
                    and Util.closest_point_on_line(Line(start, target), point) != target:
                left_perp_point = point
                left_perp_point_dist = dist

        assert left_perp_point is not None
        plan = straight_line_planner(start, left_perp_point, obstacles, MODE_LEFT) + straight_line_planner(left_perp_point, target, obstacles, MODE_LEFT)
        print(plan)
        return plan

    elif mode is MODE_RIGHT:
        obstacle_group = get_group_of_points(first_collision, obstacles)
        right_perp_point = None
        right_perp_point_dist = -1

        #perp() function returns to the clockwise side
        for ob in obstacle_group:
            point = ob.origin + (target - start).perp().norm(ob.radius + NEW_POINT_BUFFER)
            dist = Util.dist_point_to_line(point, Line(start, target))

            # value is negative is point is on "right" of line looking from start to target
            if Util.point_is_to_right_of_line(start, target, point) and (right_perp_point is None or dist > right_perp_point_dist)\
                    and Util.closest_point_on_line(Line(start, target), point) != start\
                    and Util.closest_point_on_line(Line(start, target), point) != target:
                right_perp_point = point
                right_perp_point_dist = dist

        assert right_perp_point is not None
        plan = straight_line_planner(start, right_perp_point, obstacles, MODE_RIGHT) + straight_line_planner(right_perp_point, target, obstacles, MODE_RIGHT)
        print(plan)
        return plan

    elif mode is MODE_BOTH:
        # All obstacles the path collides with. Can be 1 or more obstacles
        obstacle_group = get_group_of_points(first_collision, obstacles)
        left_point = Util.get_leftmost_point(start, target, obstacle_group, NEW_POINT_BUFFER)
        right_point = Util.get_rightmost_point(start, target, obstacle_group, NEW_POINT_BUFFER)

        left_path = straight_line_planner(start, left_point, obstacles, MODE_LEFT) + straight_line_planner(left_point, target, obstacles, MODE_BOTH)
        right_path = straight_line_planner(start, right_point, obstacles, MODE_RIGHT) + straight_line_planner(right_point, target, obstacles, MODE_BOTH)
        plan = get_shortest_path(left_path, right_path)
        print(plan)
        return plan
    else:
        assert False

def get_group_of_points(obstacle, all_obstacles):
    """
    Returns obstacle and all obstacles it touches in a list
    """
    touching_obstacles = [obstacle]

    while True:
        to_add = []
        for p in touching_obstacles:
            for o in all_obstacles:
                if p.origin.distance_to(o.origin) < p.radius + o.radius and o not in touching_obstacles:
                    to_add.append(o)

        touching_obstacles += to_add

        if len(to_add) == 0:
            break

    return touching_obstacles

def get_first_collision(start, end, obstacles):
    """
    Returns None if the path is not blocked, otherwise returns the closest obstacle that blocks it
    """
    closest_obstacle = None
    for ob in obstacles:
        if Util.intersects(Line(start, end), ob.origin, ob.radius):
            if (closest_obstacle is None) or ((ob.origin - start).length() < (closest_obstacle.origin - start).length()):
                closest_obstacle = ob

    return closest_obstacle

def get_path_with_fewest_segments(path1, path2):
    if len(path1) < len(path2):
        return path1
    elif len(path1) > len(path2):
        return path2
    else:
        return get_shortest_path(path1, path2)

def get_shortest_path(path1, path2):
    if get_dist(path1) < get_dist(path2):
        return path1
    return path2

def get_dist(path):
    dist = 0.0
    for i in range(len(path)-1):
        dist += (path[i] - path[i+1]).length()
    return dist


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
# Plotting and visualization
# Plots the robots and their avoid distacnes (as outlines) on the field, the start and endpoints, and the path
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

fig, ax = plt.subplots()
ax.grid()

# Plot the field
field1 = plt.Rectangle((-FIELD_LENGTH / 2, -FIELD_WIDTH / 2), FIELD_LENGTH, FIELD_WIDTH, lw=2, fill=False, ec='black')
field2 = plt.Rectangle((-FIELD_LENGTH / 2 - GOAL_DEPTH, -GOAL_WIDTH / 2), GOAL_DEPTH, GOAL_WIDTH, lw=2, fill=False, ec='black')
field3 = plt.Rectangle((FIELD_LENGTH / 2, -GOAL_WIDTH / 2), GOAL_DEPTH, GOAL_WIDTH, lw=2, fill=False, ec='black')
field4 = plt.Rectangle((0, -FIELD_WIDTH / 2), 0, FIELD_WIDTH, lw=2, fill=False, ec='black')
field5 = plt.Circle((0, 0), radius=CENTER_CIRCLE_RADIUS, lw=2, fill=False, ec='black')

ax.add_patch(field1)
ax.add_patch(field2)
ax.add_patch(field3)
ax.add_patch(field4)
ax.add_patch(field5)
ax.set_ylim(-FIELD_WIDTH / 2 - 0.5, FIELD_WIDTH / 2 + 0.5)
ax.set_xlim(-FIELD_LENGTH / 2 - 0.5, FIELD_LENGTH / 2 + 0.5)

# Plot the robots
for r in robots:
    ax.add_patch(plt.Circle((r.origin.x, r.origin.y), ROBOT_RADIUS, color="red"))
    ax.add_patch(plt.Circle((r.origin.x, r.origin.y), r.radius, color="red", fill=False))

# Plot the start and end points
ax.add_patch(plt.Circle((start_.x, start_.y), ROBOT_RADIUS, color="green"))
ax.add_patch(plt.Circle((target_.x, target_.y), ROBOT_RADIUS, color="green"))

# Try calculate the paths, report an error if failed (likely a stack overflow)
path_closest_side = []
path_shortest_dist = []
path_fewest_segments = []

try:
    path_closest_side = [start_] + straight_line_planner(start_, target_, robots, MODE_BOTH)
    path_shortest_dist = [start_]  # + straight_line_planner_shortest_dist(start_, target_, robots)
    path_fewest_segments = [start_]  # + straight_line_planner_fewest_segments(start_, target_, robots)
except RuntimeError as re:
    print("Sorry, but the straight line navigator could not find a valid path to the target: {}".format(re.args[0]))

pcs, = plt.plot([p.x for p in path_closest_side], [p.y for p in path_closest_side], 'g-', lw=3, label='closest side')
for p in path_closest_side:
    ax.add_patch(plt.Circle((p.x, p.y), ROBOT_RADIUS, color="green", fill=False, alpha=0.4))

psd, = plt.plot([p.x for p in path_shortest_dist], [p.y for p in path_shortest_dist], 'b-', lw=3, label='shortest dist')
for p in path_shortest_dist:
    ax.add_patch(plt.Circle((p.x, p.y), ROBOT_RADIUS, color="blue", fill=False, alpha=0.4))

pfs, = plt.plot([p.x for p in path_fewest_segments], [p.y for p in path_fewest_segments], 'm-', lw=3, label='fewest segments')
for p in path_fewest_segments:
    ax.add_patch(plt.Circle((p.x, p.y), ROBOT_RADIUS, color="magenta", fill=False, alpha=0.4))

plt.legend([pcs, psd, pfs], ['Path taking closest side of obstacle', 'Path taking shortest dist', 'Path with fewest segments/collisions'])

plt.show()
