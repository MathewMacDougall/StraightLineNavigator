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

robots = [Circle(Point(0, 0), ROBOT_AVOID_DIST), Circle(Point(0, -0.3), ROBOT_AVOID_DIST), Circle(Point(-2, -0.0), ROBOT_AVOID_DIST),
          Circle(Point(-2, -0.4), ROBOT_AVOID_DIST), Circle(Point(2, -0.2), ROBOT_AVOID_DIST), Circle(Point(0.2, 0.4), ROBOT_AVOID_DIST),
          Circle(Point(-2.7, -0.0), ROBOT_AVOID_DIST), Circle(Point(-2.7, -0.3), ROBOT_AVOID_DIST), Circle(Point(-2.7, 0.3), ROBOT_AVOID_DIST),
          Circle(Point(-3, -0.6), ROBOT_AVOID_DIST), Circle(Point(-3.4, -0.7), ROBOT_AVOID_DIST),
          Circle(Point(-3.9, -0.7), ROBOT_AVOID_DIST), Circle(Point(-4.3, -0.7), ROBOT_AVOID_DIST),
          Circle(Point(-3.1, 0.55), ROBOT_AVOID_DIST)]

start_ = Point(-3, 0)
target_ = Point(3, 0)

# fig, ax = plt.subplots()
# ax.grid()
#
# # Plot the robots
# for r in robots[6:]:
#     ax.add_patch(plt.Circle((r.origin.x, r.origin.y), ROBOT_RADIUS, color="red"))
#     ax.add_patch(plt.Circle((r.origin.x, r.origin.y), r.radius, color="red", fill=False))
#
# tangents = Util.get_group_tangent_point(start_, robots[6:])
# plt.plot([tangents[0].x], [tangents[0].y], 'r*', lw=6)
# plt.plot([tangents[1].x], [tangents[1].y], 'b*', lw=6)
#
# left_point_target = Util.get_leftmost_point(start_, target_, robots[6:], NEW_POINT_BUFFER)
# right_point_target = Util.get_rightmost_point(start_, target_, robots[6:], NEW_POINT_BUFFER)
# plt.plot([left_point_target.x], [left_point_target.y], 'r*', lw=6)
# plt.plot([right_point_target.x], [right_point_target.y], 'b*', lw=6)
#
# plt.plot([start_.x], [start_.y], 'g*', lw=6)
# plt.show()
# exit()


# define the modes for the planning algorithm
MODE_LEFT = "MODE_LEFT"
MODE_RIGHT = "MODE_RIGHT"
MODE_BOTH = "MODE_BOTH"

# THIS IS V2, the right left mode version
def straight_line_planner(start, target, obstacles, mode, maxDepth = 100):
    # print(maxDepth)
    if maxDepth < 0:
        return []

    first_collision = Util.get_first_collision(start, target, obstacles)
    if first_collision is None:
        return [target]

    # in MODE_RIGHT the planner with try go to the left (relative to the start -> target line aka counterclockwise) to avoid obstacles
    if mode is MODE_LEFT:
        obstacle_group = Util.get_group_of_points(first_collision, obstacles)
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
        plan_first_part = straight_line_planner(start, left_perp_point, obstacles, MODE_LEFT)
        plan_second_part = straight_line_planner(left_perp_point, target, obstacles, MODE_LEFT)
        if len(plan_first_part) == 0 or len(plan_second_part) == 0:
            return []
        else:
            return plan_first_part + plan_second_part

    # in MODE_RIGHT the planner with try go to the right (relative to the start -> target line aka clockwise) to avoid obstacles
    elif mode is MODE_RIGHT:
        obstacle_group = Util.get_group_of_points(first_collision, obstacles)
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
        plan_first_part = straight_line_planner(start, right_perp_point, obstacles, MODE_RIGHT)
        plan_second_part = straight_line_planner(right_perp_point, target, obstacles, MODE_RIGHT)
        if len(plan_first_part) == 0 or len(plan_second_part) == 0:
            return []
        else:
            return plan_first_part + plan_second_part

    # in MODE_BOTH, if an obstacle is encountered the planner with check paths that go around it both to the right
    # and left, and returning the best one
    elif mode is MODE_BOTH:
        # All obstacles the path collides with. Can be 1 or more obstacles
        obstacle_group = Util.get_group_of_points(first_collision, obstacles)
        left_point_target = Util.get_leftmost_point(start, target, obstacle_group, NEW_POINT_BUFFER)
        right_point_target = Util.get_rightmost_point(start, target, obstacle_group, NEW_POINT_BUFFER)
        left_point_start = Util.get_group_tangent_point(start, obstacle_group)[0]
        right_point_start = Util.get_group_tangent_point(start, obstacle_group)[1]

        left_path_1 = straight_line_planner(start, left_point_start, obstacles, MODE_RIGHT)
        left_path_2 = straight_line_planner(left_point_start, left_point_target, obstacles, MODE_RIGHT)
        left_path_3 = straight_line_planner(left_point_target, target, obstacles, MODE_BOTH)
        right_path_1 = straight_line_planner(start, right_point_start, obstacles, MODE_LEFT)
        right_path_2 = straight_line_planner(right_point_start, target, obstacles, MODE_LEFT)
        right_path_3 = straight_line_planner(right_point_target, target, obstacles, MODE_BOTH)

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
                return get_shortest_path(left_path_1 + left_path_2 + left_path_3, right_path_1 + right_path_2 + right_path_3)
                # return get_path_with_fewest_segments(left_path_1 + left_path_2 + left_path_3, right_path_1 + right_path_2 + right_path_3)

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
    print("dist left: {}     dist right: {}".format(get_dist(path1), get_dist(path2)))
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
