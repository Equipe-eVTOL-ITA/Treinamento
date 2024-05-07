class Vector2(object):
    """
    Represents a bidimensional geometric vector.
    """
    def __init__(self, x, y):
        """
        Creates a bidimensional geometric vector.

        :param x: x coordinate.
        :type x: float
        :param y: y coordinate.
        :type y: float
        """
        self.x = x
        self.y = y


class Pose(object):
    """
    Represents a pose on the plane, i.e. a (x, y) position plus a rotation.
    """
    def __init__(self, x, y, rotation):
        """
        Creates a pose on the plane.

        :param x: x coordinate.
        :type x: float
        :param y: y coordinate.
        :type y: float
        :param rotation: rotation around z axis.
        :type rotation: float
        """
        self.position = Vector2(x, y)
        self.rotation = rotation

