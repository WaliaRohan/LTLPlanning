# Write code to perform delauny traingulation on environment

# TODO: How do we provide the environment?

# From: 

import numpy as np

points = np.array([[0, 0], [0, 1.1], [1, 0], [1, 1]])

from scipy.spatial import Delaunay

tri = Delaunay(points)

# TODO How do we create vector fields from the output of delaunay?


