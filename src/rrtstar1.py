from points_container import PriorityQueue
from collision import collision
from utils import randomPoint, inside, distance, normalize, scale
import pygame as pg
import drawing
import events
import time
import config

MAX_INTERACTION = 5000

def rrtstar1(start, goal, obstacles):
    """
    start -- point (x, y)
    goal  -- point (x, y)
    obstacles: pygame.Surface
    """
    parent = {start: None}
    cost = {start: 0}

    MAX_LOOKUP_DISTANCE = 50  # Adjust the value as needed
    container = PriorityQueue()
    container.insert(start, cost[start])
    algo = 'RRT* const step'
    height = 0
    nodes = 1
    delta = 0
    current = start

    startTime = time.perf_counter()
    loop = 0
    notfound = 0
    while not inside(current, goal):
        if not events.rrtHandler():  # handle user events.
            return None

        if drawing.showInfo:  # drawing-related.
            elapsed = time.perf_counter() - startTime
            drawing.updateInfo(algo, elapsed, nodes, height)
            drawing.update()

        random_point = randomPoint(start, goal, 0.2)  # Generate a random point

        # Find the nearest point in the container to the random point
        nearest = container.NNS(random_point)
        nearest_dist = distance(nearest, random_point)

        # Generate a new point along the direction of the nearest point
        direction = normalize(random_point[0] - nearest[0], random_point[1] - nearest[1])
        new_point = (nearest[0] + scale(direction, MAX_LOOKUP_DISTANCE)[0],
                     nearest[1] + scale(direction, MAX_LOOKUP_DISTANCE)[1])

        # Check if the new point is within the boundaries
        new_point = (max(0, min(new_point[0], config.WIDTH)), max(0, min(new_point[1], config.HEIGHT)))
        if new_point[0] == start[0] and new_point[1] == start[1]:
            continue
        if new_point == nearest:  # do not allow two identical points.
            continue
        if not collision(new_point, nearest, obstacles):
            if(parent[nearest] == new_point):
                continue
            notfound = 0
            MAX_LOOKUP_DISTANCE = 50
            container.insert(new_point, cost[nearest] + distance(new_point, nearest))
            parent[new_point] = nearest
            cost[new_point] = cost[nearest] + distance(new_point, nearest)

            height = max(height, cost[new_point])
            nodes += 1
            stop = time.perf_counter()
            drawing.addEdge((nearest, new_point))
            delta += time.perf_counter() - stop

            current = new_point

            # Check if there is a direct path from the new point to the goal
            if not collision(new_point, goal, obstacles):
                parent[new_point] = nearest
                container.insert(goal, cost[new_point] + distance(new_point, goal))
                parent[goal] = new_point
                cost[goal] = cost[new_point] + distance(new_point, goal)

                height = max(height, cost[goal])
                nodes += 1
                stop = time.perf_counter()
                drawing.addEdge((new_point, goal))
                delta += time.perf_counter() - stop
                break

        notfound += 1
        if notfound > 10:
            MAX_LOOKUP_DISTANCE *= 0.9

    elapsed = time.perf_counter() - startTime - delta
    drawing.updateInfo(algo, elapsed, nodes, height, cost[goal])
    return parent