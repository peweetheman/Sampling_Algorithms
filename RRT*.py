from Graph import Graph
from Node import Node
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import math
import copy

#maybe excessively tracking by keeping track of graph edges and node parents, same for basic RRT, only small memory waste probably
class RRT_star():
	def __init__(self, initial, goal, space, growth, obstacle_list):
		"""
		:param initial: starting [x,y]
		:param self.goal: end self.goal [x.y]
		:param self.space : [min sample, max sample] (assuming square)
		:param self.growth : distance to grow new_node sample
		:param self.obstacle_list : list of obstacles
		"""
		self.initial = initial
		self.goal = goal
		self.space = space
		self.growth = growth
		self.obstacle_list = obstacle_list
		
	def RRT_star(self):
		start = Node(self.initial[0], self.initial[1])
		end = Node(self.goal[0], self.goal[1])
		last = end
		graph = Graph()
		graph.add_vertex(start)
		for i in range(0, maxIter):
			sample = sample_free()
			nearest = nearest_node(graph, sample)
			new_node = steer(nearest, sample)

			if collision_check(new_node, self.obstacle_list):
				continue

			near = find_near_nodes(graph, new_node, self.growth)
			new_node = connect_min_path(new_node, near, graph, self.obstacle_list)
			graph.add_vertex(new_node)
			rewire(new_node, near, graph, self.obstacle_list)


			# check if reached self.goal
			dx = new_node.x - end.x
			dy = new_node.y - end.y
			d = math.sqrt(dx * dx + dy * dy)
			if d <= self.growth:
				last = new_node
				break

		path = get_path(last)
		draw_graph(graph, start, end, path, self.obstacle_list)
		return [graph, path]

	def sample_free(self):
		minSample = self.space[0] #min bound on space
		maxSample = self.space[1] #max bound on space
		sample = Node(random.uniform(minSample, maxSample),
					random.uniform(minSample, maxSample))
		return sample

	def nearest_node(self, graph, sample):       ##using L2 Euclidean distance
		distances = [(node.x - sample.x) ** 2 + (node.y - sample.y)
				 ** 2 for node in graph.vertices()]
		index = distances.index(min(distances))
		nearest = graph.vertices()[index]
		return nearest

	def find_near_nodes(self, graph, new_node, self.growth):
		# gamma_star = 2(1+1/d) ** (1/d) volume(free)/volume(total) ** 1/d and we need gamma > gamma_star
		# for asymptotical completeness see Kalman 2011. gamma = 1 satisfies
		d = 2  # dimension of the self.space
		card = len(graph.vertices())  # cardinality of vertex set
		r = min(math.pow(math.log(card) / card, 1 / d), self.growth)
		cost_list = [cost(node, new_node) for node in graph.vertices()]
		near_indices = [cost_list.index(dist) for dist in cost_list if dist <= r]
		near = [graph.vertices()[i] for i in near_indices]
		return near

	def connect_min_path(self, new_node, near, graph, self.obstacle_list):
		if not near:
			return new_node

		cost_list = []
		for near_node in near:
			#angle = math.atan2(dy, dx)
			new_cost = cost(near_node, new_node)
			if not collision_check_on_path(near_node, new_node, self.obstacle_list):
				cost_list.append(near_node.cost + new_cost)
			else:
				cost_list.append(float("inf"))
		min_cost = min(cost_list)
		min_node = near[cost_list.index(min_cost)]

		if min_cost == float("inf"):
			print("mincost is infinity?!? blame Patrick")
			return new_node

		new_node.cost = min_cost
		new_node.parent = min_node
		graph.add_edge({min_node, new_node})
		return new_node

	def rewire(new_node, near, graph, self.obstacle_list):
		for near_node in near:
			if collision_check_on_path(new_node, near_node, self.obstacle_list):
				if (new_node.cost + cost(new_node, near_node)<near_node.cost):
					near_node.parent = new_node
					near_node.cost = new_node.cost + cost(new_node, near_node)
					graph.remove_edge({near_node, near_node.parent})
					graph.add_edge({new_node, near_node})


	def steer(nearest, sample, self.growth):
		angle = math.atan2(sample.y - nearest.y, sample.x-nearest.x)
		new_node = copy.deepcopy(nearest)
		new_node.x += math.cos(angle) * self.growth
		new_node.y += math.sin(angle) * self.growth
		return new_node

	def collision_check_on_path(node1, node2, self.obstacle_list):
		temp_node = copy.deepcopy(node1)
		dy = node2.y - node1.y
		dx = node2.x-node1.x
		d = cost(node1, node2)
		self.growth = .5
		angle = math.atan2(dy, dx)
		for i in range(int(d / self.growth)):
			temp_node.x += self.growth * math.cos(angle)
			temp_node.y += self.growth * math.sin(angle)
			if not collision_check(temp_node, self.obstacle_list):
				return True   # collision
		return False   # no collision


	def collision_check(node, self.obstacle_list):
		for (ox, oy, size) in self.obstacle_list:
			dx = ox - node.x
			dy = oy - node.y
			d = dx * dx + dy * dy
			if d <= size ** 2:
				return True  # collision

		return False #no collision

	def get_path(last):
		path = [[last.x, last.y]]
		while last.parent is not None:
			node = last
			path.append([node.x, node.y])
			last = node.parent
		return path

	def draw_graph(graph, start, end , path, self.obstacle_list):
		plt.clf()
		if rnd is not None:
			plt.plot(rnd[0], rnd[1], "^k")
		for node in graph.vertices():
			if node.parent is not None:
				plt.plot([node.x, node.parent.x], [
						 node.y, node.parent.y], "-k")
		print("path length: ", len(path))
		for i in range(1, len(path)):
			plt.plot([path[i][0], path[i-1][0]], [path[i][1], path[i - 1][1]], "-b")

		for (ox, oy, size) in self.obstacle_list:
			plt.plot(ox, oy, "og", ms=30 * size)

		plt.plot(start.x, start.y, "yH")
		plt.plot(end.x, end.y, "rH")
		plt.axis([0, 30, 0, 30])
		plt.grid(True)
		plt.title('RRT*')
		plt.pause(.01)

	def cost(node, new_node):
		return math.sqrt((node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2)

	def main():
		obstacleList = [
			(5, 5, 1),
			(3, 6, 2),
			(3, 8, 2),
			(3, 10, 2),
			(7, 5, 2),
			(9, 5, 2)
		]  # [x,y,size(radius)]
		plt.show()
		RRTstar([1, 1], [20, 15], [0, 30], .5, obstacleList, 100000)

	main()

