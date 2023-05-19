__author__ = 'João Pedro Gonçalves Moreira - jpgmoreira19@gmail.com'

from config import *
from rrt import rrt
from rrtstar import rrtstar
from rrtstar1 import rrtstar1
from astar import astar
import drawing
import events
import pygame as pg
pg.init()
pg.display.set_caption('Ba Lang Huyen')
def main():

	drawing.screen = pg.display.set_mode((WIDTH, HEIGHT))

	gameState = 'waiting'

	while True:
		event = pg.event.poll()
		mousePos = pg.mouse.get_pos()

		gameState = events.mainHandler(event, gameState, mousePos)

		if gameState == 'quit':
			return
		elif gameState == 'start-positioning':
			drawing.startPos = mousePos
		elif gameState == 'goal-positioning':
			drawing.goalPos = mousePos
		elif gameState == 'drawing':
			drawing.drawObstacle(mousePos)
		elif gameState == 'erasing':
			drawing.eraseObstacle(mousePos)
		elif gameState == 'clear':
			drawing.clearObstacles()
		elif gameState == 'save':
			drawing.saveObstacles()
		elif gameState == 'load':
			drawing.loadObstacles()
		elif gameState == 'rrt':
			drawing.clearEdgesPool()
			tree = rrt(drawing.startPos, drawing.goalPos, drawing.obstaclesSurface)
			if tree:  # A path was found:
				drawing.drawPath(tree)
				gameState = 'path-found'
			else:  # User terminated the algorithm's execution:
				gameState = 'waiting'		
		elif gameState == 'rrtstar':
			drawing.clearEdgesPool()
			tree = rrtstar(drawing.startPos, drawing.goalPos, drawing.obstaclesSurface)
			if tree:  # A path was found:
				drawing.drawPath(tree)
				gameState = 'path-found'
			else:  # User terminated the algorithm's execution:
				gameState = 'waiting'
		elif gameState == 'rrtstar1':
			drawing.clearEdgesPool()
			tree = rrtstar1(drawing.startPos, drawing.goalPos, drawing.obstaclesSurface)
			if tree:  # A path was found:
				drawing.drawPath(tree)
				gameState = 'path-found'
			else:  # User terminated the algorithm's execution:
				print("path not found")
				gameState = 'waiting'
		elif gameState == 'astar':
			drawing.clearEdgesPool()
			tree = astar(drawing.startPos, drawing.goalPos, drawing.obstaclesSurface)
			if tree:  # A path was found:
				drawing.drawPath(tree)
				gameState = 'path-found'
			else:  # User terminated the algorithm's execution:
				print("path not found")
				gameState = 'waiting'				

		drawing.update()

if __name__ == '__main__':
	main()
