import numpy as np
import matplotlib as plt
import math
import scipy as sp
import scipy.sparse.linalg as sps


# starting with nu = 0 and Neumann boundary conditions for precision matrix

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
		self.precision = np.zeros((n, n))
		self.b = np.zeros(n + p)

		for i in range(0, n):
			for j in range(0, n):
				if j == i: # vertex is itself

					# boundary conditions
					if i < cols or i%cols == 1 or i%cols == 0 or i>n-cols: # on an edge

						if i == 0 or i == cols or i == (rows-1)*cols or i == n-1: # in a corner
							self.precision[i][j] = 2 + a**2

						else:
							self.precision[i][j] = 3 + a**2

					else: # not on boundary
						self.precision[i][j] = a
				if j == i - cols or j == i + cols: # directly below or above
					self.precision[i][j] = -1
				if j == i + 1 and j % cols != 0: # j is to the right
					self.precision[i][j] = -1
				if j == i - 1 and i % cols != 0: # j is to the left
					self.precision[i][j] = -1

				self.precision[j][i] = self.precision[i][j] # might not be neccessary

		T_inv = 100 * np.eye(p)
		T = np.linalg.inv(np.matrix(T_inv))
		F = np.ones((n, p))

		upper = np.concatenate((self.precision, -self.precision @ F), axis=1)
		lower = np.concatenate((-F.T @ self.precision, F.T @ self.precision @ F + T), axis=1)
		self.full_precision = np.concatenate((upper, lower), axis=0)

		# use covariance to check
		full_cov = np.linalg.inv(self.full_precision)
		print(full_cov[104, 100:105])
		self.cov_diag = np.diag(full_cov)
		print(self.cov_diag)
		# cov = (sps.inv(self.precision))
		# print("covariance matrix: ", cov)
		# cov_diag = np.diagonal(cov)
		# print("cov_diag: ", cov_diag)


	def regression_update(self, x, y):
		"""
		:param x: agent location
		:param y: measurment
		:return:
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
	# a = np.array([[1, 2], [3, 4]])
	# b = np.array([[5, 6], [7, 8]])
	# print(np.concatenate((a, b), axis=1))
	rows = 10
	cols = 10
	grid_dim = [rows, cols]
	gmrf = GMRF_Regression(theta=[1, 1], grid_dim=grid_dim, f=[1, 1, 1, 1, 1], var=5)

	x = (5, 5)   # agent location
	y = 3   # measurement
	gmrf.regression_update(x, y)


main()