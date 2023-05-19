from points_container import pointsContainer
from collision import collision
from utils import randomPoint, inside, distance
import pygame as pg
import drawing
import events
import time
import heapq
from config import *

def heuristic(point, goal):
    """
    Calculate the heuristic value between two points using Euclidean distance.
    """
    x1, y1 = point
    x2, y2 = goal
    return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5

def reconstructPath(parent, goal):
    """
    Reconstruct the path from the start to the goal using the parent dictionary.
    """
    path = {}
    # current = goal
    # while current is not None:
    #     # path[current] = 
    #     current = parent[current]
    # path.reverse()
    return path

def astar(start, goal, obstacles):
    """
    start -- point (x, y)
    goal  -- point (x, y)
    obstacles: pygame.Surface
    """
    openSet = []
    heapq.heappush(openSet, (0, start))
    parent = {start: None}
    gScore = {start: 0}

    algo = 'A*'
    container = pointsContainer()
    container.insert(start)
    
    height = 0
    nodes = 1
    delta = 0
    startTime = time.perf_counter()
    min_distance = 20

    while openSet:
        if not events.rrtHandler():  # handle user events.
            return None

        if drawing.showInfo:  # drawing-related.
            elapsed = time.perf_counter() - startTime
            drawing.updateInfo(algo, elapsed, nodes, height)
            drawing.update()

        _, current = heapq.heappop(openSet)

        if inside(current, goal) or not collision(current, goal, obstacles):
            gScoreNew = gScore[current] + heuristic(current, goal)
            container.insert(goal)
            parent[goal] = current
            gScore[goal] = gScoreNew
            elapsed = time.perf_counter() - startTime - delta
            drawing.updateInfo(algo, elapsed, nodes, height, gScore[goal])
            return parent

        for neighbor in container.getNeighbors(current, obstacles, 3):
            # Check if the neighbor is too close to any existing node
            close_to_existing = False
            for existing_node in container.getPoints():
                if distance(existing_node, neighbor) < min_distance:
                    close_to_existing = True
                    break
            if close_to_existing:
                continue
            gScoreNew = gScore[current] + heuristic(current, neighbor)

            if not collision(neighbor, goal, obstacles):
                container.insert(neighbor)
                parent[neighbor] = current
                gScore[neighbor] = gScoreNew
                gScoreNew = gScore[neighbor] + heuristic(neighbor, goal)
                parent[goal] = neighbor
                height = max(height, gScore[neighbor])
                nodes += 1
                elapsed = time.perf_counter() - startTime - delta
                gScoreNew = gScore[neighbor] + heuristic(neighbor, goal)
                gScore[goal] = gScoreNew
                drawing.updateInfo(algo, elapsed, nodes, height, gScore[goal])
                return parent
            
            if neighbor not in gScore or gScoreNew < gScore[neighbor]:
                container.insert(neighbor)
                parent[neighbor] = current
                gScore[neighbor] = gScoreNew

                height = max(height, gScore[neighbor])
                nodes += 1
                heapq.heappush(openSet, (gScoreNew + heuristic(neighbor, goal), neighbor))
                stop = time.perf_counter()
                drawing.addEdge((current, neighbor))
                delta += time.perf_counter() - stop

    elapsed = time.perf_counter() - startTime - delta
    drawing.updateInfo(algo, elapsed, nodes, height)

    return None