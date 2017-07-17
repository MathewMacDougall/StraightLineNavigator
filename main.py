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
# robots = [Point(), Point(0, 0.1), Point(0.0, -0.1), Point(-0.2, 0.3), Point(-0.2, -0.3), Point(2.75, 0), Point(-2.70, -0.001),
#           Point(-2.70, -0.1), Point(-2.85, 0.4), Point(-2.9, -0.4), Point(-3.1, 0.5), Point(-3.3, 0.5), Point(-3.35, -0.55),
#           Point(1.5, 0.4), Point(1.6, 0.15)]
start_ = Point(-3, 0)
target_ = Point(3, 0)

# line = Line(Point(-1, 0), Point(1, 0))
# point = Point(0.2, 0.4)
# closest_point = Util.closest_point_on_line(line, point)
# print(closest_point)
# exit()

# print("TEST: {}".format(Point(1,1).rotate_about(Point(0, 0), math.pi / 180 * 90)))
# exit()
# print((Point(-2.9, -0.4) - Point(-3.35, -0.55)).length() < 2 * TOTAL_AVOID_DIST)
# exit()

# start = Point(-1, 0)
# target = Point(1, 0)
# point = Point(0, -1)
# print((target.x - start.x) * (point.y - start.y) - (target.y - start.y) * (point.x - start.x))
# exit()

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

# THIS IS V2, the right left mode version
MODE_LEFT = "MODE_LEFT"
MODE_RIGHT = "MODE_RIGHT"
MODE_BOTH = "MODE_BOTH"
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

# circle = Circle(Point(-3, 0.32), 0.2)
# point = Point(-4, 5)
# tangents = get_circle_tangent_points(point, circle)
# l1 = [point, tangents[0]]
# l2 = [point, tangents[1]]
# print(tangents)
#
# fig, ax = plt.subplots()
# ax.grid()
#
# ax.add_patch(plt.Circle((circle.origin.x, circle.origin.y), radius=circle.radius, lw=2, fill=False, ec='black'))
# plt.plot([p.x for p in l1], [p.y for p in l1], 'g-', lw=3)
# plt.plot([p.x for p in l2], [p.y for p in l2], 'g-', lw=3)
#
# plt.show()
# exit()



# obstacles = [Circle(Point(0, 0), 1), Circle(Point(0.1, -0.5), 1), Circle(Point(-0.5, 0.3), 1.5)]
# start = Point(-1, -2.4997)
# target = Point(2.0412, 3.52)
# point_r = get_rightmost_point(start, target, obstacles)
# point_l = get_leftmost_point(start, target, obstacles)
#
# fig, ax = plt.subplots()
# ax.grid()
#
# for circle in obstacles:
#     ax.add_patch(plt.Circle((circle.origin.x, circle.origin.y), radius=circle.radius, lw=2, fill=False, ec='black'))
# plt.plot([start.x], [start.y], 'g*', lw=3)
# plt.plot([target.x], [target.y], 'g*', lw=3)
# plt.plot([point_r.x], [point_r.y], 'r*', lw=5)
# plt.plot([point_l.x], [point_l.y], 'r*', lw=5)
#
# plt.show()
# exit()

# def straight_line_planner(start, target, all_obstacles):
#     collision = get_first_collision(start, target, all_obstacles)
#     if collision is None:
#         return [target]
#     else:
#         intermediate_pts = []
#         group = get_group_of_points(collision, all_obstacles)
#         if len(group) > 1 and line_splits_group(start, target, group):
#             # This is a group of obstacles
#             group_endpoints = get_group_collision_endpoints(start, target, group)
#             if (group_endpoints[0] - start).abs_angle_between(target - start) < \
#                     (group_endpoints[1] - start).abs_angle_between(target - start):
#                 new_point = group_endpoints[1]
#             else:
#                 new_point = group_endpoints[0]
#
#             # Check that this gets us far enough past the group
#
#             moveCW = (new_point - start).clockwise(collision - new_point)
#
#             rotate_point = new_point - get_first_collision(new_point, target, group) # points from first collision to end point
#             rotate_point = rotate_point.norm(rotate_point.length() + NEW_POINT_BUFFER)
#             basepoint = get_first_collision(new_point, target, group)
#             i_point = basepoint + rotate_point
#
#             while get_first_collision(i_point, target, group) is not None:
#                 if moveCW:
#                     rotate_point = rotate_point.rotate(-math.pi / 180 * 5)
#                 else:
#                     rotate_point = rotate_point.rotate(math.pi / 180 * 5)
#
#                 i_point = basepoint + rotate_point
#                 print("going around obstacle. CW = {} and i_point: {}".format(moveCW, i_point))
#                 intermediate_pts.append(i_point)
#                 basepoint = get_first_collision(i_point, target, group)
#         else:
#             # This is NOT a group of obstacles
#             closest_point = Util.closest_point_on_line(Line(start, target), collision)
#             new_point = collision + (closest_point - collision).norm(TOTAL_AVOID_DIST_WITH_BUFFER)
#             if closest_point.distance_to(collision) < 0.00001:
#                 new_point = collision + (target - start).perp().norm(TOTAL_AVOID_DIST_WITH_BUFFER)
#
#         if len(intermediate_pts) > 0:
#             return straight_line_planner(start, new_point, all_obstacles) + intermediate_pts + straight_line_planner(intermediate_pts[len(intermediate_pts) - 1], target, all_obstacles)
#         else:
#             return straight_line_planner(start, new_point, all_obstacles) + intermediate_pts + straight_line_planner(new_point, target, all_obstacles)

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
        # for ob in all_obstacles:
        #     to_add = []
        #     for tob in touching_obstacles:
        #         if ob != tob and ob.distance_to(tob) < 2 * TOTAL_AVOID_DIST:
        #             to_add.append(ob)
        #             break
        #     touching_obstacles += to_add

            # if ob != obstacle and obstacle.distance_to(ob) < 2 * TOTAL_AVOID_DIST:
            #     touching_obstacles.append(ob)

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
        if (cw_point - start).clockwise(p - start) and not (start - target).clockwise(p - start):
            cw_point = p
            continue
        if not (ccw_point - start).clockwise(p - start) and (start - target).clockwise(p - start):
            ccw_point = p
            continue

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
    ax.add_patch(plt.Circle((r.origin.x, r.origin.y), ROBOT_RADIUS, color="red"))
    ax.add_patch(plt.Circle((r.origin.x, r.origin.y), r.radius, color="red", fill=False))

ax.add_patch(plt.Circle((start_.x, start_.y), ROBOT_RADIUS, color="green"))
ax.add_patch(plt.Circle((target_.x, target_.y), ROBOT_RADIUS, color="green"))

try:
    path_closest_side = [start_] + straight_line_planner(start_, target_, robots, MODE_BOTH)
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