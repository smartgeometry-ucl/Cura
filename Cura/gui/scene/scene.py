__author__ = 'Jaime van Kessel'

class Scene(object):
	'''
	Base scene class. Holds all objects (all objects on platform, etc) in the 3D world.
	'''
	def __int__(self):
		self._machine = None #Scene has a reference to the machine