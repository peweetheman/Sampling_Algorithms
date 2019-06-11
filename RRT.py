##RRT algorithm basic
import numpy;
from .Graph import Graph


def buildRRT(n):
	graph = Graph("root")
	random = sampleFree()
	nearest = nearestNode(graph, random)
	new = steer(nearest, random)
	graph.add_vertex(new)
	graph.add_edge({nearest, new})

def sampleFree():
	x = randomX()
	y = randomY()
	sample = {x,y}
	return sample

def nearestNode(graph, random):       ##using euclidian distance right now
	minDistance = (graph.vertices[0].x - random.x)^2 + (graph.vertices[0].y - random.y)^2
	nearest_node = graph.vertices[0]
	for v in graph.vertices():
		distance = (v.x - random.x)^2 + (v.y - random.y)^2
		if distance < minDistance:
			nearest_node = v
	return nearest_node

def steer(nearest, random):

	return nearest
	