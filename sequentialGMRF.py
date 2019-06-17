import numpy as np
import matplotlib as plt
import math
import scipy as sp
import scipy.sparse.linalg as sps


#starting with nu = 0
class GMRF_Regression:
	
	def __init__(self, theta, grid_dim, f, var):
		"""
		:param theta:
		:param grid_dim:
		:param f:
		:param var:
		"""

		self.theta = theta
		self.grid_dim = grid_dim
		self.f = f
		self.var = var

		rows = self.grid_dim[0]
		cols = self.grid_dim[1]
		n = rows * cols
		p = len(self.f)

		t = self.theta[0]
		k = self.theta[1]
		a = k ** 2 + 4

		self.precision = np.zeros((n + p, n + p))
		self.b = np.zeros(n + p)

		for i in range(0, n):
			for j in range(0, n):
				print(i, j)
				if j == i:
					self.precision[i][j] = a
				if j == i - cols:
					self.precision[i][j] = -1
				if j == i + 1 and i != cols:
					self.precision[i][j] = -1

				self.precision[j][i] = self.precision[i][j]

		print(self.precision)
		cov = (sps.inv(self.precision))
		print("covariance matrix: ", cov)
		cov_diag = np.diagonal(cov)
		print("cov_diag: ", cov_diag)


	def regression(self):
		"""
		:param self.theta: hyperparemters
		:param grid:
		:param f:
		:param var:
		"""


	def phi(self, x, grid):
		"""
		:param x: location of measurement
		:param grid: grid
		:return: phi
		"""
		vertices = get_vertices(x, grid)
		phi_1 = 1/(ab) * (x_e - a/2) * (y_e - b/2)
		return phi
	
	def get_vertices(self, x, grid):
		"""
	
		:param x:
		:param grid:
		:return: four x,y coordinates of nearby vertices
		"""
		vertices = [[]]
		return vertices

def main():
	rows = 10
	cols = 10
	grid_dim = [rows, cols]
	gmrf = GMRF_Regression(theta=[1, 1], grid_dim=grid_dim, f=[1], var=5)
	gmrf.regression()


main()