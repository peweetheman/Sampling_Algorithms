from Graph import Graph
from Node import Node
import random
import matplotlib.pyplot as plt
import math
import copy


def RRT(initial, goal, space, growth, obstacles):
	"""
	:param initial: starting [x,y]
	:param goal: end goal [x.y]
	:param space : [min sample, max sample] (assuming square)
	:param growth : distance to grow new sample
	:return: RRT graph and path from start to end
	"""
	start = Node(initial[0], initial[1])
	end = Node(goal[0], goal[1])
	graph = Graph()
	graph.add_vertex(start)
	while True:
		sample = sample_free(space[0], space[1])
		index = nearest_node(graph, sample)
		nearest = graph.vertices()[index]
		new = steer(nearest, sample, growth)

		#check collision
		if collision_check(new, obstacles):
			continue

		new.parent = nearest
		graph.add_vertex(new)
		graph.add_edge({nearest, new})

		# check if reached end
		dx = new.x - end.x
		dy = new.y - end.y
		d = math.sqrt(dx * dx + dy * dy)
		if d <= growth:
			last = new
			break
	# reconstruct path
	path = [[end.x, end.y]]
	while last.parent is not None:
		node = last
		path.append([node.x, node.y])
		last = node.parent
	draw_graph(graph, start, end, path)
	return [graph, path]

def sample_free(minSample, maxSample):
	sample = Node(random.uniform(minSample, maxSample),
				random.uniform(minSample, maxSample))
	return sample

def nearest_node(graph, sample):       ##using L2 euclidian distance right now
	distances = [(node.x - sample.x) ** 2 + (node.y - sample.y)
			 ** 2 for node in graph.vertices()]
	nearestIndex = distances.index(min(distances))
	return nearestIndex

def steer(nearest, sample, growth):
	angle = math.atan2(sample.y - nearest.y, sample.x-nearest.x)
	new = copy.deepcopy(nearest)
	new.x += math.cos(angle) * growth
	new.y += math.sin(angle) * growth
	return new

def collision_check(node, obstacles):
	return False

def draw_graph(graph, start, end , path):
	"""
	Draws graph
	"""
	plt.clf()
	for node in graph.vertices():
		if node.parent is not None:
			plt.plot([node.x, node.parent.x], [
					 node.y, node.parent.y], "-k")
	print("path length: ", len(path))
	for i in range(1, len(path)):
		plt.plot([path[i][0], path[i-1][0]], [path[i][1], path[i - 1][1]], "-b")
	plt.plot(start.x, start.y, "yH")
	plt.plot(end.x, end.y, "rH")
	plt.axis([0, 20, 0, 20])
	plt.grid(True)
	plt.title('RRT')
	plt.show()

def main():
	plt.show()
	RRT([1, 1], [10, 15], [0, 20], .5, None)

main()

