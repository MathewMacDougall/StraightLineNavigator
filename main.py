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

robots = [Point(), Point(0, 0.1), Point(0.0, -0.1), Point(-0.2, 0.3), Point(-0.2, -0.3), Point(2.75, 0), Point(-2.70, -0.001),
          Point(-2.70, -0.1), Point(-2.85, 0.4),# Point(-2.9, -0.4), Point(-3.1, 0.5),
          Point(1.5, 0.4), Point(1.6, 0.15)]
start_ = Point(-3, 0)
target_ = Point(3, 0)


# IDEAS TO OPTIMIZE:
# - shortest distance
# - fewest obstacles / fewest segments
# - some combo of the 2

# Takes the closest side
# def straight_line_planner(start, target, all_obstacles, cw = False):
#     collision = get_first_collision(start, target, all_obstacles)
#     if collision is None:
#         return [target]
#     else:
#         group = get_group_of_points(collision, all_obstacles)
#         if len(group) > 1 and line_splits_group(start, target, group):
#             # This is a group of obstacles
#             group_endpoints = get_group_collision_endpoints(start, target, group)
#             if (group_endpoints[0] - start).abs_angle_between(target - start) < \
#                     (group_endpoints[1] - start).abs_angle_between(target - start):
#                 new_point = group_endpoints[0]
#             else:
#                 new_point = group_endpoints[1]
#         else:
#             # This is NOT a group of obstacles
#             closest_point = Util.closest_point_on_line(Line(start, target), collision)
#             new_point = collision + (closest_point - collision).norm(TOTAL_AVOID_DIST_WITH_BUFFER)
#             if closest_point.distance_to(collision) < 0.00001:
#                 new_point = collision + (target - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER)
#
#         return straight_line_planner(start, new_point, all_obstacles) + straight_line_planner(new_point, target, all_obstacles)

def straight_line_planner(start, target, all_obstacles):
    collision = get_first_collision(start, target, all_obstacles)
    if collision is None:
        return [target]
    else:
        group = get_group_of_points(collision, all_obstacles)
        if len(group) > 1 and line_splits_group(start, target, group):
            # This is a group of obstacles
            group_endpoints = get_group_collision_endpoints(start, target, group)
            if (group_endpoints[0] - start).abs_angle_between(target - start) < \
                    (group_endpoints[1] - start).abs_angle_between(target - start):
                new_point = group_endpoints[0]
            else:
                new_point = group_endpoints[1]
        else:
            # This is NOT a group of obstacles
            closest_point = Util.closest_point_on_line(Line(start, target), collision)
            new_point = collision + (closest_point - collision).norm(TOTAL_AVOID_DIST_WITH_BUFFER)
            if closest_point.distance_to(collision) < 0.00001:
                new_point = collision + (target - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER)

        return straight_line_planner(start, new_point, all_obstacles) + straight_line_planner(new_point, target, all_obstacles)


# def straight_line_planner(start, target, all_obstacles, cw = False):
#     collision = get_first_collision(start, target, all_obstacles)
#     if collision is None:
#         return [target]
#     else:
#         group = get_group_of_points(collision, all_obstacles)
#         if len(group) > 1:
#             group_endpoints = get_group_collision_endpoints(start, target, group)
#         else:
#             closest_point = Util.closest_point_on_line(Line(start, target), collision)
#             group_endpoints = [collision + (closest_point - collision).norm(TOTAL_AVOID_DIST_WITH_BUFFER),
#                                collision - (closest_point - collision).norm(TOTAL_AVOID_DIST_WITH_BUFFER)]
#             if closest_point.distance_to(collision) < 0.00001:
#                 group_endpoints = [collision + (target - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER),
#                                    collision - (target - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER)]
#
#         path1 = straight_line_planner(start, group_endpoints[0], all_obstacles) + straight_line_planner(group_endpoints[0], target, all_obstacles)
#         path2 = straight_line_planner(start, group_endpoints[1], all_obstacles) + straight_line_planner(group_endpoints[1], target, all_obstacles)
#         return get_shortest_path(path1, path2)
#
#         # if len(group) > 1 and line_splits_group(start, target, group):
#         #     # This is a group of obstacles
#         #     group_endpoints = get_group_collision_endpoints(start, target, group)
#         #     if (group_endpoints[0] - start).abs_angle_between(target - start) < \
#         #             (group_endpoints[1] - start).abs_angle_between(target - start):
#         #         new_point = group_endpoints[0]
#         #     else:
#         #         new_point = group_endpoints[1]
#         # else:
#         #     # This is NOT a group of obstacles
#         #     closest_point = Util.closest_point_on_line(Line(start, target), collision)
#         #     new_point = collision + (closest_point - collision).norm(TOTAL_AVOID_DIST_WITH_BUFFER)
#         #     if closest_point.distance_to(collision) < 0.00001:
#         #         new_point = collision + (target - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER)
#         #
#         # return straight_line_planner(start, new_point, all_obstacles) + straight_line_planner(new_point, target, all_obstacles)


def get_group_of_points(obstacle, all_obstacles):
    """
    Returns obstacle and all obstacles it touches in a list
    """
    touching_obstacles = [obstacle]
    for ob in all_obstacles:
        if ob != obstacle and obstacle.distance_to(ob) < 2 * TOTAL_AVOID_DIST:
            touching_obstacles.append(ob)

    return touching_obstacles


def line_splits_group(start, target, obstacle_group):
    """
    Returns true if the line specified by start->target goes between any 2 obstacles in the obstacle group
    """
    cw = False
    ccw = False
    for p in obstacle_group:
        if (target - start).clockwise(p - start):
            cw = True
        if not (target - start).clockwise(p - start):
            ccw = True

    return cw and ccw


def get_group_collision_endpoints(start, target, obstacle_group):
    """
    Returns the two edge points of the obstacle_group that are tangentially furthest
    from the start->target line
    """
    cw_point = obstacle_group[0]
    ccw_point = obstacle_group[0]
    for p in obstacle_group:
        if (cw_point - start).clockwise(p - start):
            cw_point = p
        if not (ccw_point - start).clockwise(p - start):
            ccw_point = p

    cw_endpoint = cw_point + (cw_point - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER)
    ccw_endpoint = ccw_point - (ccw_point - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER)
    print("endpoints (cw, ccw) are: {} and {}".format(cw_endpoint, ccw_endpoint))
    return [cw_endpoint, ccw_endpoint]


def get_first_collision(start, end, obstacles):
    """
    Returns None if the path is not blocked, otherwise returns the position of the closest obstacle that blocks it
    """
    closest_obstacle = None
    for ob in obstacles:
        if Util.intersects(Line(start, end), ob, TOTAL_AVOID_DIST):
            if (closest_obstacle is None) or ((ob - start).length() < (closest_obstacle - start).length()):
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


fig, ax = plt.subplots()
ax.grid()

# Plot the field
field1 = plt.Rectangle((-FIELD_LENGTH / 2, -FIELD_WIDTH / 2), FIELD_LENGTH, FIELD_WIDTH, lw=2, fill=False, ec='black')
field2 = plt.Rectangle((-FIELD_LENGTH / 2 - GOAL_DEPTH, -GOAL_WIDTH / 2), GOAL_DEPTH, GOAL_WIDTH, lw=2, fill=False,
                       ec='black')
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

for r in robots:
    ax.add_patch(plt.Circle((r.x, r.y), ROBOT_RADIUS, color="red"))
    ax.add_patch(plt.Circle((r.x, r.y), TOTAL_AVOID_DIST, color="red", fill=False))

ax.add_patch(plt.Circle((start_.x, start_.y), ROBOT_RADIUS, color="green"))
ax.add_patch(plt.Circle((target_.x, target_.y), ROBOT_RADIUS, color="green"))

try:
    path_closest_side = [start_] + straight_line_planner(start_, target_, robots)
except RuntimeError as re:
    print("Sorry, but the straight line navigator could not find a valid path to the target: {}".format(re.args[0]))


path_shortest_dist = [start_] #+ straight_line_planner_shortest_dist(start_, target_, robots)
path_fewest_segments = [start_] #+ straight_line_planner_fewest_segments(start_, target_, robots)

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


print("Total avoid dist: {}".format(TOTAL_AVOID_DIST))
# print(Point(1, 5).angle() - Point(1, 0).angle())
# print(Point(5, 0).angle() - Point(1, 5).angle())
# print(Point(-1, 5).angle())
# print(Point(-1, -1).angle())
# print(Point(0, -6).angle())
# print(Point(1, -2).angle())
# print(Point(0, 1).clockwise(Point(1, 0)))
# print(Point(-5, 1).clockwise(Point(-1, 1)))
# print(Point(-5, 1).clockwise(Point(-1, -1)))
# print(Point(-0.6, -1.55).clockwise(Point(1, -1.0444)))