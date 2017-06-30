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

robots = [Point(), Point(0, -0.3), Point(1, 0.2), Point(1.5, -0.2), Point(-2.75, 0), Point(2.75, 0)]
start_ = Point(-3, 0)
target_ = Point(3, 0)

# IDEAS TO OPTIMIZE:
# - shortest distance
# - fewest obstacles / fewest segments
# - some combo of the 2
def straight_line_planner_fewest_segments(start, target, obstacles):
    return []
    collision = get_first_collision(start, target, obstacles)
    if collision is None:
        return [target]
    else:
        closest_point = Util.closest_point_on_line(Line(start, target), collision)
        new_point = collision + (closest_point - collision).norm(TOTAL_AVOID_DIST + NEW_POINT_BUFFER)
        if closest_point.distance_to(collision) < 0.00001:
            new_point = collision + (target - start).perp().norm(TOTAL_AVOID_DIST + NEW_POINT_BUFFER)

        # mirror the new point to the other side of the obstacle
        new_point2 = Point(new_point.x, collision.y - (new_point.y - collision.y))

        return get_path_with_fewest_segments(straight_line_planner(start, new_point, obstacles) + straight_line_planner(new_point, target, obstacles),
                                             straight_line_planner(start, new_point2, obstacles) + straight_line_planner(new_point2, target, obstacles))

def straight_line_planner_shortest_dist(start, target, obstacles):
    return []
    collision = get_first_collision(start, target, obstacles)
    if collision is None:
        return [target]
    else:
        closest_point = Util.closest_point_on_line(Line(start, target), collision)
        new_point = collision + (closest_point - collision).norm(TOTAL_AVOID_DIST + NEW_POINT_BUFFER)
        if closest_point.distance_to(collision) < 0.00001:
            new_point = collision + (target - start).perp().norm(TOTAL_AVOID_DIST + NEW_POINT_BUFFER)

        # mirror the new point to the other side of the obstacle
        new_point2 = Point(new_point.x, collision.y - (new_point.y - collision.y))

        return get_shortest_path(straight_line_planner(start, new_point, obstacles) + straight_line_planner(new_point, target, obstacles),
                                 straight_line_planner(start, new_point2, obstacles) + straight_line_planner(new_point2, target, obstacles))

# Takes the closest side
def straight_line_planner(start, target, obstacles, lastCollision = None):
    collision = get_first_collision(start, target, obstacles)
    if collision is None:
        return [target]
    else:
        closest_point = Util.closest_point_on_line(Line(start, target), collision)
        new_point = collision + (closest_point - collision).norm(TOTAL_AVOID_DIST + NEW_POINT_BUFFER)
        if closest_point.distance_to(collision) < 0.00001:
            new_point = collision + (target - start).perp().norm(TOTAL_AVOID_DIST + NEW_POINT_BUFFER)

        if get_first_collision(start, new_point, obstacles) is None:
            return straight_line_planner(start, new_point, obstacles) + straight_line_planner(new_point, target, obstacles)
        else:
            





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

path_closest_side = [start_] + straight_line_planner(start_, target_, robots)
path_shortest_dist = [start_] + straight_line_planner_shortest_dist(start_, target_, robots)
path_fewest_segments = [start_] + straight_line_planner_fewest_segments(start_, target_, robots)

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
# print(Util.closest_point_on_line(Line(Point(-3, 0), Point(3, 0)), Point(2.55, 1)))
print(Util.intersects(Line(Point(2.758072309789024, 0.2498696416427374), Point(3, 0)), Point(2.75, 0), TOTAL_AVOID_DIST))
# print(Util.dist_point_to_line(Point(2.7, 0), Line(Point(2.7, 0), Point(3, 0))))
# (2.708072309789022, 0.24986964164273748)
# (2.758072309789024, 0.2498696416427374)