from Graph import Graph
from Node import Node
import random
import matplotlib.pyplot as plt
import math
import copy


def RRTstar(initial, goal, space, growth, obstacles, maxIter):
	"""
	:param initial: starting [x,y]
	:param goal: end goal [x.y]
	:param space : [min sample, max sample] (assuming square)
	:param growth : distance to grow new sample
	:param obstacles : list of obstacles
	:return: RRT graph and path from start to end
	"""
	start = Node(initial[0], initial[1])
	end = Node(goal[0], goal[1])
	graph = Graph()
	graph.add_vertex(start)
	for i in range(0, maxIter):
		sample = sample_free(space[0], space[1])
		nearest = nearest_node(graph, sample)
		new = steer(nearest, sample, growth)
		#check collision
		if collision_check(new, obstacles):
			continue

		near = find_near_nodes(graph, new, min(gamma, growth))
		graph.add_vertex(new)

		minNode = nearest
		cmin = cost(nearest) + cost#min cost



		new.parent = minNode
		graph.add_edge({minNode, new})

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

def nearest_node(graph, sample):       ##using L2 Euclidean distance
	distances = [(node.x - sample.x) ** 2 + (node.y - sample.y)
			 ** 2 for node in graph.vertices()]
	index = distances.index(min(distances))
	nearest = graph.vertices()[index]
	return nearest

def find_near_nodes(graph, new):
	d = 2       # dimension of the space
	card = len(graph.vertices()) # cardinality of vertex set
	r = math.pow(math.log(card)/card, 1/d)
	dlist = [math.sqrt((node.x - new.x) ** 2 +
			 (node.y - new.y)) ** 2 for node in graph.vertices()]
	near = [dlist[i] for i in range(0, len(dlist)) if dlist[i] <= r]
	return near

def rewire(new, near, graph):
	card = len(graph.vertices()) # cardinality of vertex set
	for node in near:


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
	RRTstar([1, 1], [10, 15], [0, 20], .5, None, 100000)

main()

