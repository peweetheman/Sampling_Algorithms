import numpy as np
import matplotlib as plt
import math


class GMRFregression():

	def __init__(self, theta, grid, extended_grid, f, var):
		"""
		:param theta: hyperparemters
		:param grid:
		:param extended_grid:
		:param f:
		:param var:
		"""
		self.theta = theta
		self.grid = grid
		self.extended_grid = extended_grid
		self. f = f
		self.var = var
		self.t = theta[0]
		self.k = theta[1]
		self.b = np.array ####################
		a = self.k^2 + 4
		grid_dim = len(grid)
		precision = (4+a^2)*self.t*np.eye(grid_dim, grid_dim)
		for i in range(0, len(grid)):
			for j in range(0, len(grid)):
				precision[i][j] = 1/(4+self.a^2)

	def extend(grid):
		"""
		:param grid: a NxN grid of random variables
		:return: a N+1 x N+1 grid of random variables?
		"""
		grid_star = 5
		return grid_star

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