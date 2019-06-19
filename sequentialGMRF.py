import numpy as np
from true_field import true_field
from Vertex import Vertex
from matplotlib import pyplot as plt
import math
import scipy as sp
from scipy.stats import multivariate_normal
import scipy.sparse.linalg as sps


# starting with nu = 0 and Neumann boundary conditions for precision matrix

class GMRF_Regression:
	
	def __init__(self, theta, grid, f, var):
		"""
		:param theta:
		:param grid_dim:
		:param f:
		:param var:
		"""
		self.num_agents=1
		self.theta = theta
		self.grid = grid
		self.f = f
		self.var = var
		#  grid.shape = num dimensions, first dimension length, second dimension length...
		self.rows = grid.shape[1]
		self.cols = grid.shape[2]
		n = self.rows * self.cols
		p = len(self.f)

		t = self.theta[0]
		k = self.theta[1]
		a = k ** 2 + 4
		self.precision = np.zeros((n, n))
		self.b = np.zeros(n + p)

		for i in range(0, n):
			for j in range(0, n):
				if j == i:   # vertex is itself

					# boundary conditions
					if i < self.cols or i % self.cols == 1 or i % self.cols == 0 or i > n-self.cols:   # on an edge

						if i == 0 or i == self.cols or i == (self.rows-1)*self.cols or i == n-1:   # in a corner
							self.precision[i][j] = 2 + a**2

						else:
							self.precision[i][j] = 3 + a**2

					else: # not on boundary
						self.precision[i][j] = a
				if j == i - self.cols or j == i + self.cols:  # directly below or above
					self.precision[i][j] = -1
				if j == i + 1 and j % self.cols != 0:  # j is to the right
					self.precision[i][j] = -1
				if j == i - 1 and i % self.cols != 0:  # j is to the left
					self.precision[i][j] = -1

				self.precision[j][i] = self.precision[i][j]  # might not be neccessary

		T_inv = 100 * np.eye(p)
		T = np.linalg.inv(np.matrix(T_inv))
		F = np.ones((n, p))

		upper = np.concatenate((self.precision, -self.precision @ F), axis=1)
		lower = np.concatenate((-F.T @ self.precision, F.T @ self.precision @ F + T), axis=1)
		self.full_precision = np.concatenate((upper, lower), axis=0)

		# use covariance to check
		full_cov = np.linalg.inv(self.full_precision)
		self.cov_diag = np.diag(full_cov)
		self.precision = self.full_precision
		# print("cov diag: ", self.cov_diag)
		# cov = (sps.inv(self.precision))
		# print("covariance matrix: ", cov)
		# cov_diag = np.diagonal(cov)
		# print("cov_diag: ", cov_diag)

	def regression_update(self, locations, measurements):
		for k in range(0, len(locations)):

			phi_k = self.compute_phi(locations[k], self.grid)
			print(measurements[k])
			print("at location: ", locations[k])
			self.b = self.b + phi_k.T * measurements[k]
			self.precision = self.precision + 1 / self.var * phi_k.T @ phi_k

			h = np.linalg.inv(self.precision) @ phi_k.T

			# print("shape phi: ", phi_k.shape)
			# print("shape precision: ", np.linalg.inv(self.precision).shape)
			# print("shape h : ", h.shape)

			self.cov_diag = self.cov_diag - (h @ h.T) / (self.var + phi_k @ h)
		mu = self.precision @ self.b
		print(mu.shape)
		return mu

	def compute_phi(self, location, grid):
		"""
		ASSUMING UNIFORM, REGULAR GRID SPACING, ON INTEGER VALUES (1,1), (1,2) etc.
		:param x: location of measurement
		:param grid: grid
		:return: phi
		"""
		x, y = location[0], location[1]
		grid_spacing = grid[1][1][2] - grid[1][1][1]
		a = grid_spacing
		b = grid_spacing

		vertices = self.get_vertices(x, y, grid_spacing)
		center = Vertex(vertices[0].x + grid_spacing/2, vertices[0].y + grid_spacing/2)
		x_e = x - center.x
		y_e = y - center.y

		phi_temp = np.zeros(4)

		phi_temp[0] = 1/(a*b) * (x_e - a/2) * (y_e - b/2)
		phi_temp[1] = -1/(a*b) * (x_e + a/2) * (y_e - b/2)
		phi_temp[2] = 1/(a*b) * (x_e + a/2) * (y_e + b/2)
		phi_temp[3] = -1/(a*b) * (x_e - a/2) * (y_e + b/2)

		phi = np.zeros((self.rows * self.cols + len(self.f),1))

		for i in range(len(phi_temp)-1, -1, -1):
			phi[self.cols * vertices[i].y + vertices[i].x] = phi_temp[i]
		phi = phi.T
		return phi

	def get_vertices(self, x, y, grid):
		"""
		:param x:
		:param grid:
		:return: tuple of four tuples of closest vertices.
		Starting with -x, -y, then -y, +x, then +x,+y, then -x, +y as seen in page 17, Andre Rene Geist master thesis
		"""
		v1 = Vertex(math.floor(x), math.floor(y))
		v2 = Vertex(math.ceil(x), math.floor(y))
		v3 = Vertex(math.ceil(x), math.ceil(y))
		v4 = Vertex(math.floor(x), math.ceil(y))
		vertices = v1, v2, v3, v4
		return vertices


def main():
	field = true_field()
	# a = np.array([[1, 2], [3, 4]])
	# b = np.array([[5, 6], [7, 8]])
	# print(np.concatenate((a, b), axis=1))
	x, y = np.mgrid[0:30:1, 0:30:1]
	grid = np.array([x, y])

	gmrf = GMRF_Regression(theta=[1, 1], grid=grid, f=[1, 1, 1, 1, 1], var=5)

	locations = np.array([(1.3, 1.5), (2.5, 2.2), (3, 3), (4, 4), (5, 5)])
	measurements = field.get_measurement(locations.T)  # measurements
	mus=gmrf.regression_update(locations, measurements)

	plt.subplot(211)
	field.draw(plt)
	plt.subplot(212)

	rv = multivariate_normal(mus, gmrf.cov)
	plt.pcolormesh(self.xi, self.yi, self.zi.reshape(self.xi.shape))

	#plt.subplots_adjust(bottom=0.1, right=0.8, top=0.9)
	plt.show()

main()