from points_container import pointsContainer
from collision import collision
from utils import randomPoint, inside
import pygame as pg
import drawing
import events
import time

def rrt(start, goal, obstacles):
	"""
	start -- point (x, y)
	goal  -- point (x, y)
	obstacles: pygame.Surface
	"""
	parent = { start: None }
	depth = { start: 0 }

	container = pointsContainer()
	container.insert(start)
	
	height = 0
	nodes = 1
	delta = 0
	current = start
	algo = 'RRT'
	startTime = time.perf_counter()

	while not inside(current, goal):
		if not events.rrtHandler():  # handle user events.
			return None

		if drawing.showInfo:  # drawing-related.
			elapsed = time.perf_counter() - startTime
			drawing.updateInfo(algo, elapsed, nodes, height)
			drawing.update()

		sample = randomPoint(start, goal, 0)
		nearest = container.NNS(sample)

		if (sample == nearest):  # do not allow two identical points.
			continue
		
		if not collision(sample, nearest, obstacles):
			container.insert(sample)
			parent[sample] = nearest
			depth[sample] = depth[nearest] + 1

			height = max(height, depth[sample])
			nodes += 1
			stop = time.perf_counter()
			drawing.addEdge( (nearest, sample) )
			delta += time.perf_counter() - stop
			current = sample
		# Check if there is a direct path from the new point to the goal
		if not collision(current, goal, obstacles):
			parent[goal] = current
			depth[goal] = depth[current] + 1
			height = max(height, depth[goal])
			nodes += 1
			drawing.addEdge((current, goal))
			current = goal
			break
	if not goal in parent:
		parent[goal] = current
		depth[goal] = depth[current] + 1
		height = max(height, depth[goal])
		nodes += 1
		stop = time.perf_counter()
		drawing.addEdge( (current, goal) )
		delta += time.perf_counter() - stop

	elapsed = time.perf_counter() - startTime - delta
	drawing.updateInfo(algo, elapsed, nodes, height, depth[goal])

	return parent
