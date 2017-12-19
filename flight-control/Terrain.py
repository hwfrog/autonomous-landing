import numpy as np


class Terrain:
	def __init__(self):
		self.caffemodle = None

	def safe_region(self, image):
		#input: an image
		#return: a matix showing safety; 1 for safe, 0 for not  
		return np.zeros((3,3))