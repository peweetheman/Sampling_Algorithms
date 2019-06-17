# libraries
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import kde


class true_field():

	def __init__(self):
		# create data
		np.random.seed(1924)
		# Generate 200 correlated x,y points
		data = np.random.multivariate_normal([0, 0], [[1, 0.5], [0.5, 3]], 200)
		x, y = data.T

		# Evaluate a gaussian kde on a regular grid of nbins x nbins over data extents
		nbins = 300
		self.k = kde.gaussian_kde([x, y])

		xi, yi = np.mgrid[x.min():x.max():nbins * 1j, y.min():y.max():nbins * 1j]
		zi = self.k(np.vstack([xi.flatten(), yi.flatten()]))

		# Make the plot
		# plt.show()
		#
		# Change color palette
		# plt.pcolormesh(xi, yi, zi.reshape(xi.shape), cmap=plt.cm.Greens_r)
		# plt.show()

		# Add color bar
		#plt.pcolormesh(xi, yi, zi.reshape(xi.shape), cmap=plt.cm.Greens_r)
		plt.pcolormesh(xi, yi, zi.reshape(xi.shape))
		plt.colorbar()

	def draw(self):
		plt.show()

	def get_value(self, x, y):
		return self.k.evaluate((x, y))

	def get_covariance(self):
		return self.k.covariance


def main():
	field = true_field()
	print(field.get_value(0, 0))
	field.draw()

main()
