import matplotlib.pyplot as plt
from Geom import *

# Constants for the field/simulation
FIELD_LENGTH = 9
FIELD_WIDTH = 6
CENTER_CIRCLE_RADIUS = 0.5
GOAL_WIDTH = 1
GOAL_DEPTH = 0.02

ROBOT_RADIUS = 0.09
NEW_POINT_BUFFER = 0.02
ROBOT_AVOID_DIST = 2 * ROBOT_RADIUS + 0.1

# robots = [Circle(Point(0, 0), ROBOT_AVOID_DIST * 3), Circle(Point(0, -0.3), ROBOT_AVOID_DIST), Circle(Point(-2, -0.0), ROBOT_AVOID_DIST),
#           Circle(Point(-2, 1.0), ROBOT_AVOID_DIST * 2), Circle(Point(2, -0.2), ROBOT_AVOID_DIST), Circle(Point(0.2, 0.8), ROBOT_AVOID_DIST),
#           Circle(Point(-2.7, -0.0), ROBOT_AVOID_DIST), Circle(Point(-2.7, -0.3), ROBOT_AVOID_DIST), Circle(Point(-2.7, 0.3), ROBOT_AVOID_DIST),
#           Circle(Point(-3, -0.6), ROBOT_AVOID_DIST), Circle(Point(-3.4, -0.7), ROBOT_AVOID_DIST),
#           Circle(Point(-3.5, -0.2), ROBOT_AVOID_DIST), Circle(Point(-3.1, 0.55), ROBOT_AVOID_DIST)]

robots = [Circle(Point(-2.5, 0), ROBOT_AVOID_DIST), Circle(Point(-2.5, 1.5), ROBOT_AVOID_DIST * 4.5), Circle(Point(-2.5, -0.3), ROBOT_AVOID_DIST),
          Circle(Point(0, -0.3), ROBOT_AVOID_DIST * 4.5)]

start_ = Point(-3, 0)
target_ = Point(3, 1)

# define the modes for the planning algorithm
MODE_LEFT = "MODE_LEFT"
MODE_RIGHT = "MODE_RIGHT"
MODE_BOTH = "MODE_BOTH"
MODE_CLOSEST_SIDE = "MODE_CLOSEST_SIDE"
DIST = "OPTIMIZE_DIST"
SEG = "OPTIMIZE_SEG"

# THIS IS V2, the right left mode version
def straight_line_planner(start, target, obstacles, mode, maxDepth, optimize):
    if maxDepth < 0:
        return []

    first_collision = Util.get_first_collision(start, target, obstacles)
    if first_collision is None:
        return [target]

    # In MODE_LEFT the path will attempt to go left around an obstacle (this generates new points to the left of the
    # current start -> target line
    if mode is MODE_LEFT:
        obstacle_group = Util.get_group_of_points(first_collision, obstacles)
        left_perp_point = None
        left_perp_point_dist = -1

        # perp() function returns to the clockwise side
        for ob in obstacle_group:
            point = ob.origin - (target - start).perp().norm(ob.radius + NEW_POINT_BUFFER)
            dist = Util.dist_point_to_line(point, Line(start, target))

            if not Util.point_is_to_right_of_line(start, target, point) and (
                    left_perp_point is None or dist > left_perp_point_dist) \
                    and Util.closest_point_on_line(Line(start, target), point) != start \
                    and Util.closest_point_on_line(Line(start, target), point) != target:
                left_perp_point = point
                left_perp_point_dist = dist

        assert left_perp_point is not None
        plan_first_part = straight_line_planner(start, left_perp_point, obstacles, MODE_LEFT, maxDepth - 1, optimize)
        plan_second_part = straight_line_planner(left_perp_point, target, obstacles, MODE_LEFT, maxDepth - 1, optimize)
        if len(plan_first_part) == 0 or len(plan_second_part) == 0:
            return []
        else:
            return plan_first_part + plan_second_part

    # In MODE_RIGHT the path will attempt to go right around an obstacle (this generates new points to the right of the
    # current start -> target line
    elif mode is MODE_RIGHT:
        obstacle_group = Util.get_group_of_points(first_collision, obstacles)
        right_perp_point = None
        right_perp_point_dist = -1

        #perp() function returns to the clockwise side
        for ob in obstacle_group:
            point = ob.origin + (target - start).perp().norm(ob.radius + NEW_POINT_BUFFER)
            dist = Util.dist_point_to_line(point, Line(start, target))

            if Util.point_is_to_right_of_line(start, target, point) and (right_perp_point is None or dist > right_perp_point_dist)\
                    and Util.closest_point_on_line(Line(start, target), point) != start\
                    and Util.closest_point_on_line(Line(start, target), point) != target:
                right_perp_point = point
                right_perp_point_dist = dist

        assert right_perp_point is not None
        plan_first_part = straight_line_planner(start, right_perp_point, obstacles, MODE_RIGHT, maxDepth - 1, optimize)
        plan_second_part = straight_line_planner(right_perp_point, target, obstacles, MODE_RIGHT, maxDepth - 1, optimize)
        if len(plan_first_part) == 0 or len(plan_second_part) == 0:
            return []
        else:
            return plan_first_part + plan_second_part

    # In MODE_BOTH, if an obstacle is encountered the planner with check paths that go around it both to the right
    # and left, and returning the best one
    elif mode is MODE_BOTH:
        obstacle_group = Util.get_group_of_points(first_collision, obstacles)
        left_point_start = Util.get_group_tangent_points(start, obstacle_group, NEW_POINT_BUFFER)[0]
        right_point_start = Util.get_group_tangent_points(start, obstacle_group, NEW_POINT_BUFFER)[1]
        left_point_target = Util.get_group_tangent_points(target, obstacle_group, NEW_POINT_BUFFER)[1]
        right_point_target = Util.get_group_tangent_points(target, obstacle_group, NEW_POINT_BUFFER)[0]

        test_collision_left = Util.get_first_collision(start, left_point_start, obstacles)
        if test_collision_left is not None and test_collision_left not in obstacle_group:
            left_path_1 = straight_line_planner(start, left_point_start, obstacles, MODE_CLOSEST_SIDE, maxDepth - 1, optimize)
        else:
            left_path_1 = straight_line_planner(start, left_point_start, obstacles, MODE_BOTH, maxDepth - 1, optimize)
        left_path_2 = straight_line_planner(left_point_start, left_point_target, obstacles, MODE_LEFT, maxDepth - 1, optimize)
        left_path_3 = straight_line_planner(left_point_target, target, obstacles, MODE_BOTH, maxDepth - 1, optimize)

        test_collision_right = Util.get_first_collision(start, right_point_start, obstacles)
        if test_collision_right is not None and test_collision_right not in obstacle_group:
            right_path_1 = straight_line_planner(start, right_point_start, obstacles, MODE_CLOSEST_SIDE, maxDepth - 1, optimize)
        else:
            right_path_1 = straight_line_planner(start, right_point_start, obstacles, MODE_BOTH, maxDepth - 1, optimize)
        right_path_2 = straight_line_planner(right_point_start, right_point_target, obstacles, MODE_RIGHT, maxDepth - 1, optimize)
        right_path_3 = straight_line_planner(right_point_target, target, obstacles, MODE_BOTH, maxDepth - 1, optimize)

        if len(left_path_1) == 0 or len(left_path_2) == 0 or len(left_path_3) == 0:
            if len(right_path_1) == 0 or len(right_path_2) == 0 or len(right_path_3) == 0:
                # neither direction is valid
                return []
            else:
                # only the right path is valid so use it
                return right_path_1 + right_path_2 + right_path_3
        else:
            if len(right_path_1) == 0 or len(right_path_2) == 0 or len(right_path_3) == 0:
                # only the left path is valid so use it
                return left_path_1 + left_path_2 + left_path_3
            else:
                # both left and right paths are valid so pick the shortest one
                if optimize is SEG:
                    return get_path_with_fewest_segments([start] + left_path_1 + left_path_2 + left_path_3, [start] + right_path_1 + right_path_2 + right_path_3)
                else:
                    # When comparing two paths for the shortest one the start point MUST be included
                    return get_shortest_path([start] + left_path_1 + left_path_2 + left_path_3, [start] + right_path_1 + right_path_2 + right_path_3)

    elif mode is MODE_CLOSEST_SIDE:
        closest_point = Util.closest_point_on_line(Line(start, target), first_collision.origin)
        closest_point = first_collision.origin + (closest_point - first_collision.origin).norm(first_collision.radius + NEW_POINT_BUFFER)
        path1 = straight_line_planner(start, closest_point, obstacles, MODE_CLOSEST_SIDE, maxDepth - 1, optimize)
        path2 = straight_line_planner(closest_point, target, obstacles, MODE_CLOSEST_SIDE, maxDepth - 1, optimize)
        if len(path1) == 0 or len(path2) == 0:
            return []
        else:
            return path1 + path2

    else:
        assert False

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
# Plots the robots and their avoid distances (as outlines) on the field, the start and endpoints, and the path
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
    path_shortest_dist = [start_] + straight_line_planner(start_, target_, robots, MODE_BOTH, 20, DIST)
    path_fewest_segments = [start_] #+ straight_line_planner(start_, target_, robots, MODE_BOTH, 20, SEG)

    if path_shortest_dist == [start_] or path_fewest_segments == [start_]:
        print("The straight line planner reached its max recursive depth and has failed to plan a full path")
    print("shortest path dist: {}    other path dist: {}".format(get_dist(path_shortest_dist), get_dist(path_fewest_segments)))
    print("fewest segment path segments: {}    other path segments: {}".format(len(path_fewest_segments), len(path_shortest_dist)))
except RuntimeError as re:
    print("Sorry, but the straight line navigator could not find a valid path to the target: {}".format(re.args[0]))

psd, = plt.plot([p.x for p in path_shortest_dist], [p.y for p in path_shortest_dist], 'b-', lw=3, label='shortest dist')
for p in path_shortest_dist:
    ax.add_patch(plt.Circle((p.x, p.y), ROBOT_RADIUS, color="blue", fill=False, alpha=0.4))

pfs, = plt.plot([p.x for p in path_fewest_segments], [p.y for p in path_fewest_segments], 'm-', lw=3, label='fewest segments')
for p in path_fewest_segments:
    ax.add_patch(plt.Circle((p.x, p.y), ROBOT_RADIUS, color="magenta", fill=False, alpha=0.4))

plt.legend([psd, pfs], ['Path taking shortest dist', 'Path with fewest segments/collisions'])

plt.show()
