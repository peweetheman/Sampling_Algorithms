import numpy as np


class Node(object):

	def __init__(self, pose):
		self.pose = np.array(pose)
		self.parent = None
		self.cost = 0.0

	def __len__(self):
		return len(self.pose)

	def __getitem__(self, i):
		return self.pose[i]

	def __repr__(self):
		return 'Node({}, {}, {})'.format(self.pose[0], self.pose[1], self.cost)

	def __eq__(self, other):
		return self.__dict__ == other.__dict__
