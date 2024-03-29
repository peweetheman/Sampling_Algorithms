import numpy as np
from true_field import true_field
from Vertex import Vertex
from matplotlib import pyplot as plt
import math
import scipy as sp
from scipy.stats import multivariate_normal
import scipy.sparse as sps
import scipy.sparse.linalg as spsl
import time


# starting with nu = 0 and Neumann boundary conditions for simple precision matrix

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
		#  grid.shape = first dimension length, second dimension length..., num dimensions
		self.rows = grid.shape[0]
		self.cols = grid.shape[1]
		n = self.rows * self.cols
		p = len(self.f)

		t = self.theta[0]
		k = self.theta[1]
		a = k ** 2 + 4
		self.precision = np.zeros((n, n))
		self.b = np.zeros(n + p).reshape(n+p, 1)

		for i in range(0, n):
			for j in range(0, n):
				if j == i:   # vertex is itself
					# boundary conditions
					if i < self.cols or i % self.cols == 1 or i % self.cols == 0 or i > n-self.cols:   # on an edge

						if i == 0 or i == self.cols or i == (self.rows-1)*self.cols or i == n-1:   # in a corner
							self.precision[i][j] = 2 + a**2

						else:                        # just on an edge
							self.precision[i][j] = 3 + a**2
					else:     # not on boundary
						self.precision[i][j] = a
				if j == i - self.cols or j == i + self.cols:  # directly below or above in grid
					self.precision[i][j] = -1
				if j == i + 1 and j % self.cols != 0:  # j is to the right in grid
					self.precision[i][j] = -1
				if j == i - 1 and i % self.cols != 0:  # j is to the left in grid
					self.precision[i][j] = -1

				self.precision[j][i] = self.precision[i][j]  # might not be necessary

		T = 1/100 * np.eye(p)
		F = np.ones((n, p))

		upper = np.concatenate((self.precision, -self.precision @ F), axis=1)
		lower = np.concatenate((-F.T @ self.precision, F.T @ self.precision @ F + T), axis=1)
		self.full_precision = np.concatenate((upper, lower), axis=0)

		# use covariance to check
		full_cov = np.linalg.inv(self.full_precision)
		self.cov_diag = np.diag(full_cov)
		self.precision = self.full_precision
		self.sparse_precision = sps.csc_matrix(self.precision)

		# print("cov diag: ", self.cov_diag)
		# print("covariance matrix: ", cov)
		# cov_diag = np.diagonal(cov)
		# print("cov_diag: ", cov_diag)

	def regression_update(self, locations, measurements):
		for k in range(0, len(locations)):
			phi_k = self.compute_phi(locations[k], self.grid)
			print(measurements[k])
			# print("at location: ", locations[k])
			self.b = self.b + phi_k.T * measurements[k]
			self.sparse_precision += sps.csc_matrix(1 / self.var * phi_k.T @ phi_k)
			#h = spsl.spsolve(self.sparse_precision, phi_k.T)
			#self.cov_diag = self.cov_diag - (h @ h.T) / (self.var + phi_k @ h)      # conditional variance

			mu = spsl.spsolve(self.sparse_precision, self.b)
					#draw precision matrix
			# plt.clf()
			#
			# x, y = np.mgrid[0:900:1, 0:900:1]
			# grid = np.dstack((x, y))
			# grid_points = grid.reshape(len(x) * len(y), 2)
			# prec = np.zeros((900, 900))
			# for [xi, yi] in grid_points:
			# 	prec[xi][yi] = self.precision[xi][yi]
			# plt.title("precision matrix")
			# plt.pcolormesh(x, y, prec.reshape(x.shape))
			# plt.colorbar()
			# plt.show()

						#draw field and variance
			# x, y = np.mgrid[0:30:1, 0:30:1]
			# grid = np.dstack((x, y))
			# grid_points = grid.reshape(len(x) * len(y), 2)
			# z = np.zeros((30, 30))
			# var = np.zeros((30, 30))
			# for [xi, yi] in grid_points:
			# 	z[xi][yi] = mu[self.cols * yi + xi]
			# 	var[xi][yi] = self.cov_diag[self.cols * yi + xi]
			# plt.subplot(2, 2, 1)
			# plt.title("learned field")
			# plt.pcolormesh(x, y, z.reshape(x.shape))
			# plt.colorbar()
			# plt.subplot(2, 2, 2)
			# plt.title("variance field")
			# plt.pcolormesh(x, y, var.reshape(x.shape))
			# plt.colorbar()
			# plt.show()

		#mu = spsl.inv(self.sparse_precision) @ self.b
		return mu, self.cov_diag

	def compute_phi(self, location, grid):
		"""
		ASSUMING UNIFORM, REGULAR GRID SPACING, ON INTEGER VALUES (1,1), (1,2) etc.
		:param x: location of measurement
		:param grid: grid
		:return: phi
		"""
		x, y = location[0], location[1]
		grid_spacing = grid[1][2][1] - grid[1][1][1]
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
	start_time = time.time()

	step_size = 1   # dimension of one side of grid

	field = true_field(step_size=step_size)
	x, y = np.mgrid[0:30:1, 0:30:1]
	grid = np.dstack((x, y))
	grid_points = grid.reshape(len(x)*len(y), 2)

	gmrf = GMRF_Regression(theta=[1, 1], grid=grid, f=[5, 5, 5, 5], var=2.5)

	locations = np.array(grid_points)
	measurements = field.get_measurement(locations.T)  # measurements
	mu, conditional_var = gmrf.regression_update(locations, measurements)

	plt.subplot(2, 2, 3)
	plt.title("true field")
	field.draw(plt)

	z = np.zeros((30, 30))
	var = np.zeros((30, 30))
	for [xi, yi] in grid_points:
		z[xi][yi] = mu[gmrf.cols * yi + xi]
		var[xi][yi] = conditional_var[gmrf.cols * yi + xi]
		# FOR VALUE COMPARISONS AT GRID POINTS
	# for i in range(0, len(field.xi)):
	# 	for j in range(0, len(field.xi)):
	# 		print(field.xi[i][j], field.yi[i][j], field.zi.reshape(field.xi.shape)[i][j])
	# print("NOW THE OTHER ONE")
	# for i in range(0, len(x)):
	# 	for j in range(0, len(field.xi)):
	# 		print(x[i][j], y[i][j], z.reshape(x.shape)[i][j])

	plt.subplot(2, 2, 1)
	plt.title("learned field")
	plt.pcolormesh(x, y, z.reshape(x.shape))
	plt.colorbar()
	plt.subplot(2, 2, 2)
	plt.title("variance field")
	plt.pcolormesh(x, y, var.reshape(x.shape))
	plt.colorbar()
	print("--- %s seconds ---" % (time.time() - start_time))
	plt.show()

main()