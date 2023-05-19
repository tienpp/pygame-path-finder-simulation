from random import randrange as rand
from config import *
import math
from random import random


def dist(p1, p2):
	"""
	Compute the euclidean distance between p1 and p2.

	p1 -- point (x, y)
	p2 -- point (x, y)
	"""
	return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def randomPoint(start, goal, bias=0.0):
    """
    Returns coordinates of a random point on the screen that prefers the straight line
    between the start and goal points.

    Args:
        start: Tuple or list representing the start point (start_x, start_y).
        goal: Tuple or list representing the goal point (goal_x, goal_y).
        bias: Bias towards the straight line (0.0 to 1.0). Higher values increase the bias.

    Returns:
        A random point that favors the straight line between the start and goal points.
    """
    start_x, start_y = start
    goal_x, goal_y = goal

    if random() < bias:
        # Generate a point along the straight line
        t = random()
        x = int(start_x + t * (goal_x - start_x))
        y = int(start_y + t * (goal_y - start_y))
    else:
        # Generate a completely random point
        x = rand(WIDTH)
        y = rand(HEIGHT)

    return x, y

def inside(point, center):
	"""
	Determine if point is inside the circle centered at
	  center and with radius equal config.RADIUS.
	"""
	return dist(point, center) < RADIUS

def normalize(vx, vy):
	"""
	Normalizes the input vector and returns its coordinates.
	"""
	norm = math.sqrt(vx * vx + vy * vy)
	if (norm > 1e-6):
		vx /= norm
		vy /= norm
	return (vx, vy)

def distance(point1, point2):
    """
    Calculates the Euclidean distance between two 2D points.

    Args:
        point1: Tuple or list representing the first point (x1, y1).
        point2: Tuple or list representing the second point (x2, y2).

    Returns:
        The Euclidean distance between the two points.
    """
    x1, y1 = point1
    x2, y2 = point2
    dx = x2 - x1
    dy = y2 - y1
    return int(math.sqrt(dx**2 + dy**2))

def scale(vector, factor):
    """
    Scale a vector by a given factor.

    Args:
        vector: Tuple or list representing the vector (x, y).
        factor: Scaling factor.

    Returns:
        Scaled vector.
    """
    x, y = vector
    return (int(x * factor), int(y * factor))