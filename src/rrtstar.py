from points_container import PriorityQueue
from collision import collision
from utils import randomPoint, inside, distance
import pygame as pg
import drawing
import events
import time

def rrtstar(start, goal, obstacles):
    """
    start -- point (x, y)
    goal  -- point (x, y)
    obstacles: pygame.Surface
    """
    parent = {start: None}
    cost = {start: 0}

    container = PriorityQueue()
    container.insert(start, cost[start])
    algo = 'RRT*'
    height = 0
    nodes = 1

    current = start
    delta = 0
    startTime = time.perf_counter()

    while not inside(current, goal):
        if not events.rrtHandler():  # handle user events.
            return None

        if drawing.showInfo:  # drawing-related.
            elapsed = time.perf_counter() - startTime
            drawing.updateInfo(algo, elapsed, nodes, height)
            drawing.update()

        sample = randomPoint(start, goal)
        nearest = container.NNS(sample)

        if sample == nearest:  # do not allow two identical points.
            continue

        if not collision(sample, nearest, obstacles):
            container.insert(sample, cost[current] + distance(sample, nearest))
            parent[sample] = nearest
            cost[sample] = cost[current] + distance(sample, nearest)

            height = max(height, cost[sample])
            nodes += 1
            stop = time.perf_counter()
            drawing.addEdge((nearest, sample))
            delta += time.perf_counter() - stop

            current = sample

        # Check if there is a direct path from the new point to the goal
        if not collision(current, goal, obstacles):
            container.insert(goal, cost[current] + distance(current, goal))
            parent[goal] = current
            cost[goal] = cost[current] + distance(current, goal)

            height = max(height, cost[goal])
            nodes += 1
            stop = time.perf_counter()
            drawing.addEdge((current, goal))
            delta += time.perf_counter() - stop

            break

    elapsed = time.perf_counter() - startTime - delta
    drawing.updateInfo(algo, elapsed, nodes, height, cost[goal])

    return parent