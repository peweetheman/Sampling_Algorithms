import numpy as np
import matplotlib as plt
import math


#starting with nu = 0

def GMRF_Regression(theta, grid_dim, f, var):
	"""
	:param theta: hyperparemters
	:param grid:
	:param extended_grid:
	:param f:
	:param var:
	"""
	t = theta[0]
	k = theta[1]
	rows = grid_dim[0]
	cols = grid_dim[1]
	b = np.zeros(rows*cols + len(f))
	a = k^2 + 4
	grid_precision = np.zeros((rows, cols))
	for i in range(0, rows):
		for j in range(0, cols):
			if i == j:
				grid_precision[i][j] = (4 + a**2) * t


def phi(x, grid):
	"""
	:param x: location of measurement
	:param grid: grid
	:return: phi
	"""
	vertices = get_vertices(x, grid)
	phi_1 = 1/(ab) * (x_e - a/2) * (y_e - b/2)
	return phi

def get_vertices(x, grid):
	"""

	:param x:
	:param grid:
	:return: four x,y coordinates of nearby vertices
	"""
	vertices = [[]]
	return vertices

def main():
	rows = 100
	cols = 100
	grid_precision = np.zeros((rows, cols))
	print(grid_precision)

main()