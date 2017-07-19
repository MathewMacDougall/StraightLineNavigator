"""Point and Rectangle classes.

Adapted from https://wiki.python.org/moin/PointsAndRectangles

Point  -- point with (x,y) coordinates
Circle -- a circle with a center-point and radius
"""

import math


class Point:
    """A point identified by (x,y) coordinates.

    supports: +, -, *, /, str, repr

    length  -- calculate length of vector to point from origin
    distance_to  -- calculate distance between two points
    as_tuple  -- construct tuple (x,y)
    clone  -- construct a duplicate
    integerize  -- convert x & y to integers
    floatize  -- convert x & y to floats
    move_to  -- reset x & y
    slide  -- move (in place) +dx, +dy, as spec'd by point
    slide_xy  -- move (in place) +dx, +dy
    rotate  -- rotate around the origin
    rotate_about  -- rotate around another point
    """

    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, p):
        """Point(x1+x2, y1+y2)"""
        return Point(self.x + p.x, self.y + p.y)

    def __sub__(self, p):
        """Point(x1-x2, y1-y2)"""
        return Point(self.x - p.x, self.y - p.y)

    def __mul__(self, scalar):
        """Point(x1*x2, y1*y2)"""
        return Point(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        """Point(x1/x2, y1/y2)"""
        return Point(self.x / scalar, self.y / scalar)

    def norm(self, scalar = 1.9):
        """Return a Point with length 'scalar'"""
        if self.length() < 0.00001:
            return Point(0.0, 0.0)
        else:
            return Point(self.x * scalar / self.length(), self.y * scalar / self.length())

    def __str__(self):
        return "(%s, %s)" % (self.x, self.y)

    def __repr__(self):
        return "%s(%r, %r)" % (self.__class__.__name__, self.x, self.y)

    def length(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def lengthsq(self):
        return self.x ** 2 + self.y ** 2

    def distance_to(self, p):
        """Calculate the distance between two points."""
        return (self - p).length()

    def as_tuple(self):
        """(x, y)"""
        return (self.x, self.y)

    def clone(self):
        """Return a full copy of this point."""
        return Point(self.x, self.y)

    def integerize(self):
        """Convert co-ordinate values to integers."""
        self.x = int(self.x)
        self.y = int(self.y)

    def floatize(self):
        """Convert co-ordinate values to floats."""
        self.x = float(self.x)
        self.y = float(self.y)

    def set(self, x, y):
        """Reset x & y coordinates."""
        self.x = x
        self.y = y

    def translate(self, dx, dy):
        """Move to new (x+dx,y+dy)"""
        self.x = self.x + dx
        self.y = self.y + dy

    def rotate(self, rad):
        """Rotate counter-clockwise by rad radians.

        Positive y goes *up,* as in traditional mathematics.

        Interestingly, you can use this in y-down computer graphics, if
        you just remember that it turns clockwise, rather than
        counter-clockwise.

        The new position is returned as a new Point.
        """
        s, c = [f(rad) for f in (math.sin, math.cos)]
        x, y = (c * self.x - s * self.y, s * self.x + c * self.y)
        return Point(x, y)

    def rotate_about(self, p, theta):
        """Rotate counter-clockwise around a point, by theta radians.

        Positive y goes *up,* as in traditional mathematics.

        The new position is returned as a new Point.
        """
        result = self.clone()
        result.translate(-p.x, -p.y)
        result = result.rotate(theta)
        result.translate(p.x, p.y)
        return result

    def perp(self):
        """
        Note: this always returns the "clockwise facing" vector to a Point/vector
        """
        return Point(self.y, -self.x)

    def dot(self, other):
        """
        returns the dot product between this and other
        """
        return (self.x * other.x) + (self.y * other.y)

    def cross(self, other):
        """
        returns the cross product between this and other
        """
        return self.x * other.y - self.y * other.x

    def clockwise(self, other):
        """
        Returns true if other is clockwise of this point
        """
        if self.y * other.x > self.x * other.y:
            return True
        else:
            return False

    def project(self, other):
        """
        Returns the projection of this point onto other
        """
        return self.dot(other) / other.lengthsq() * other

    def angle(self):
        if self.length() < 0.000000001:
            return 0

        if self.y >= 0:
            return math.acos(self.x / self.length())
        else:
            return 2 * math.pi - math.acos(self.x / self.length())

    def abs_angle_between(self, other):
        """
        Returns the absolute value of the angle between this and other in radians
        0 <= pi radians
        """
        return math.acos(self.dot(other) / (self.length() * other.length()))

class Circle:
    def __init__(self, center_init, radius_init):
        self.origin = center_init
        self.radius = radius_init

    def diameter(self):
        return 2 * self.radius

    def circumference(self):
        return math.pi * 2 * self.radius

    def area(self):
        return math.pi * (self.radius ** 2)

class Line:
    def __init__(self, p1_init, p2_init):
        self.start = p1_init
        self.end = p2_init

    def length(self):
        return self.start.distance_to(self.end)

    def midpoint(self):
        return self.start + ((self.end - self.start) / 2)

    def get_line_equation(self):
        """
        Returns the variables a, b, and c from the line equation
        ax + by + c
        Knowing the start_ and endpoint of the line we can get the equation of a line using:
        y - y1 = (y2 - y1) / (x2 - x1) * (x - x1)
        """
        a = self.start.y - self.end.y
        b = self.end.x - self.start.x
        c = (self.start.x - self.end.x) * self.start.y + self.start.x * (self.end.y - self.start.y)
        return [a, b, c]


class Util:
    @staticmethod
    def intersects(line, point, radius):
        """
        Same as above but the circle is represented by the point and radius
        """
        dist = Util.dist_point_to_line(point, line)
        if dist <= radius:
            return True
        else:
            return False

    @staticmethod
    def dist_point_to_line(point, line):
        """
        Computes the shortest distance from the point to a point on the line
        """
        seglen = line.length()
        relsecond_s = point - line.start
        relsecond_e = point - line.end

        s_vec2 = (line.end - line.start)
        s_vec2_reverse = (line.start - line.end)

        ds = s_vec2.dot(relsecond_s)
        de = s_vec2_reverse.dot(relsecond_e)
        if ds > 0.0 and de > 0.0:
            if line.length() < 0.0000001:
                return relsecond_s.length()
            cross = relsecond_s.cross(s_vec2)
            return math.fabs(cross / seglen)

        len_s = (line.start - point).length()
        len_e = (line.end - point).length()
        return min(len_s, len_e)

    @staticmethod
    def is_perp(p1, p2):
        return math.fabs(p1.dot(p2)) < 0.000001

    @staticmethod
    def closest_point_on_line(line, other_point):
        """
        Returns the point on the line closest to other_point
        """
        if line.length() < 0.000000001:
            return line.start

        lenseg = (line.end - line.start).dot(other_point - line.start) / (line.length())
        c = line.start + (line.end - line.start).norm(lenseg)

        ac = (line.start - c).length()
        bc = (line.end - c).length()
        ab = line.length()
        in_range = (ac <= ab) and (bc <= ab)

        if in_range:
            return c

        len_a = (c - line.start).length()
        len_b = (c - line.end).length()

        if len_a < len_b:
            return line.start

        return line.end

    @staticmethod
    def get_closest_endpoint(line, point):
        d1 = (line.start - point).length()
        d2 = (line.end - point).length()
        if d1 <= d2:
            return line.start
        else:
            return line.end

    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    # Helpers for the straight line navigator 2.0
    # ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @staticmethod
    def get_circle_tangent_points(line_start, circle, buffer):
        """
        Returns the points on the circle (plus a dist of buffer) that create tangent lines with line_start
        Returns points [left, right] from the perspective of line_start -> circle
        """
        radius_angle = math.acos(circle.radius / (line_start - circle.origin).length())
        p1 = (line_start - circle.origin).rotate(radius_angle).norm(circle.radius + buffer) + circle.origin
        p2 = (line_start - circle.origin).rotate(-radius_angle).norm(circle.radius + buffer) + circle.origin
        return [p1, p2]

    @staticmethod
    def get_rightmost_point(start, target, obstacles, buffer):
        leftmost_point = None
        for ob in obstacles:
            tangent_points = Util.get_circle_tangent_points(target, ob, buffer)
            t1 = tangent_points[0]
            t2 = tangent_points[1]

            if (start - target).clockwise(t1 - target) and (
                            leftmost_point is None or (t1 - target).abs_angle_between(start - target) > (
                                leftmost_point - target).abs_angle_between(start - target)):
                leftmost_point = t1
            if (start - target).clockwise(t2 - target) and (
                            leftmost_point is None or (t2 - target).abs_angle_between(start - target) > (
                                leftmost_point - target).abs_angle_between(start - target)):
                leftmost_point = t2

        assert leftmost_point is not None
        return leftmost_point

    @staticmethod
    def get_leftmost_point(start, target, obstacles, buffer):
        rightmost_point = None
        for ob in obstacles:
            tangent_points = Util.get_circle_tangent_points(target, ob, buffer)
            t1 = tangent_points[0]
            t2 = tangent_points[1]

            if not (start - target).clockwise(t1 - target) and (
                            rightmost_point is None or (t1 - target).abs_angle_between(start - target) > (
                                rightmost_point - target).abs_angle_between(start - target)):
                rightmost_point = t1
            if not (start - target).clockwise(t2 - target) and (
                            rightmost_point is None or (t2 - target).abs_angle_between(start - target) > (
                                rightmost_point - target).abs_angle_between(start - target)):
                rightmost_point = t2

        assert rightmost_point is not None
        return rightmost_point

    @staticmethod
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

    @staticmethod
    def get_first_collision(start, end, obstacles):
        """
        Returns None if the path is not blocked, otherwise returns the closest/first obstacle that blocks it
        """
        closest_obstacle = None
        for ob in obstacles:
            if Util.intersects(Line(start, end), ob.origin, ob.radius):
                if (closest_obstacle is None) or (
                        (ob.origin - start).length() < (closest_obstacle.origin - start).length()):
                    closest_obstacle = ob

        return closest_obstacle

    @staticmethod
    def point_is_to_right_of_line(start, end, point):
        """
        Returns true if the point is to the right of the line defined as start->end.
        To the right of the line means to the right looking from the perspective of start -> end
        """
        return (end.x - start.x) * (point.y - start.y) - (end.y - start.y) * (point.x - start.x) < 0

    @staticmethod
    def get_group_tangent_point(start, obstacles, buffer):
        """
        Returns the points that form tangent lines from start -> obstacles
        Returns [left tangent, right tangent], where left and right are defined from the perspective of the start
        looking towards the obstacle
        """
        t1 = None
        c1 = None
        t2 = None
        for p in obstacles:
            for t in Util.get_circle_tangent_points(start, p, buffer):
                collision = Util.get_first_collision(start, start + (t-start).norm(100), obstacles)
                if t1 is None and collision is None:
                    t1 = t
                    c1 = p
                    continue
                if t1 is not None and collision is None:
                    t2 = t
                    break

        # figure out which is the left and right tangents for the group
        # return [left, right]
        if (start - c1.origin).clockwise(t1 - c1.origin):
            return [t1, t2]
        else:
            return [t2, t1]
