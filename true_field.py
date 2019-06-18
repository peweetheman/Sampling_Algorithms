# libraries
import numpy as np
from scipy.stats import kde
from matplotlib import pyplot as plt


class true_field():

	def __init__(self):
		# create random seeded data, some correlation
		np.random.seed(4213)
		data1 = np.random.multivariate_normal(mean=[15, 15], cov=[[5, 2], [2, 3]], size=10)
		data2 = np.random.multivariate_normal(mean=[25, 5], cov=[[8, 2], [2, 7]], size=25)
		data3 = np.random.multivariate_normal(mean=[3, 15], cov=[[3, 2], [2, 4]], size=25)
		data4 = 30.0 * np.random.rand(200, 2)
		data = np.concatenate((data1, data2, data3, data4), axis=0)
		x, y = data.T

		# Evaluate a gaussian kde on a regular grid of nbins x nbins over data extents
		nbins = 300
		self.k = kde.gaussian_kde([x, y])

		self.xi, self.yi = np.mgrid[x.min():x.max():nbins * 1j, y.min():y.max():nbins * 1j]
		self.zi = 10000.0 * self.k(np.vstack([self.xi.flatten(), self.yi.flatten()]))

		# Make the plot
		# plt.show()
		#
		# Change color palette
		# plt.pcolormesh(self.xi, self.yi, self.zi.reshape(self.xi.shape), cmap=plt.cm.Greens_r)
		# plt.show()

		# Add color bar
		# plt.pcolormesh(self.xi, self.yi, self.zi.reshape(self.xi.shape), cmap=plt.cm.Greens_r)

	def draw(self, plt):
		plt.pcolormesh(self.xi, self.yi, self.zi.reshape(self.xi.shape))
		plt.colorbar()
		plt.show()

	def get_measurement(self, x, y):
		return self.k.evaluate((x, y))

	def get_covariance(self):
		return self.k.covariance


def main():
	field = true_field()
	field.draw(plt)

main()