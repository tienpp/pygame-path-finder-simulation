from utils import dist
import heapq
import random
from collision import collision
from config import *

class pointsContainer:
	"""
	An ADT to store 2D points and find a point's nearest neighbor.

	To solve the Nearest Neighbor Search problem (NNS), I chose to
	  do a linear search, since it is much simpler than other
	  approaches and produces a reasonably good result here.

	Time complexities:
	  - insert: O(1)
	  - NNS: O(N), where N is the number of points currently inside the container.
	"""
	def __init__(self):
		self._points = []
	
	def insert(self, point):
		self._points.append(point)
	
	def NNS(self, point):
		best = self._points[0]  # will throw IndexError if self._points is empty.
		bestDist = dist(best, point)
		for p in self._points:
			pDist = dist(p, point)
			if pDist < bestDist:
				best = p
				bestDist = pDist
		return best
	def getNeighbors(self, point, obstacles, num_neighbors=8, max_offset=500):
		x, y = point
		neighbors = []
		count = 0
		while count < num_neighbors:
			dx = random.randint(-max_offset, max_offset)
			dy = random.randint(-max_offset, max_offset)
			neighbor = (x + dx, y + dy)

			if((neighbor[0] < 0) or (neighbor[0] > WIDTH) or (neighbor[1] > HEIGHT) or (neighbor[1] < 0) or collision(neighbor, point, obstacles)):
				continue
			neighbors.append(neighbor)
			count += 1

		return neighbors



class PriorityQueue:
    """
    An ADT to store items with priorities and retrieve the item with the highest priority.

    This implementation uses a min-heap to efficiently retrieve the item with the highest priority.

    Time complexities:
      - insert: O(log N), where N is the number of items currently in the queue.
      - pop: O(log N)
      - __len__: O(1)
    """

    def __init__(self):
        self._queue = []
        self._index = 0

    def insert(self, item, priority):
        heapq.heappush(self._queue, (priority, self._index, item))
        self._index += 1

    def pop(self):
        return heapq.heappop(self._queue)[-1]

    def __len__(self):
        return len(self._queue)

    def NNS(self, point):
        best = self._queue[0][-1]  # will throw IndexError if self._queue is empty.
        bestDist = dist(best, point)
        for _, _, p in self._queue:
            pDist = dist(p, point)
            if pDist < bestDist:
                best = p
                bestDist = pDist
        return best